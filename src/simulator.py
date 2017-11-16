#!/usr/bin/env python
import neat, visualize
import os, pickle, sys
import rospy
import math, random

from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from walkyto.srv import *
from std_msgs.msg import String
from std_srvs.srv import Empty

from apply_joint_effort_client import *
from spawn_model_client import *
from delete_model_client import *

class Twist :
	def __init__(self, linear, angular):
		self.linear = linear
		self.angular = angular

class Vector3 :
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z

class simulator:

	def gazebo_init(self):		
		local_dir = os.getenv("GAZEBO_MODEL_PATH")
		sdf_file =  open('%s/%s/model.sdf'%(local_dir, self.model_name), 'r')
		model_xml = sdf_file.read()
		model_name = '%s_%d' % (self.model_name, self.dup_num)
		robot_namespace = model_name

		pos = position(0, (self.dup_num-1)*18 - (self.total_dup-1)*9, 0)
		orient = orientation(0, 0, 0, 0)
		initial_pose = pose(pos, orient)
		reference_frame=''

		spawn_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)

	def gazebo_exit(self):
		delete_model('%s_%d' % (self.model_name, self.dup_num))

	def set_world_state(self, state):
		model_name = '%s_%d' % (self.model_name, self.dup_num)

		for key in list(self.link_reference.keys()):
			link_name = key
			reference_name = self.link_reference[key]
			joint_name = '%s::%s'%(model_name,link_name)
			ref_joint_name = '%s::%s'%(model_name,reference_name)

			if joint_name in state.name:
				pose = state.pose[state.name.index(joint_name)]
				twist = Twist(Vector3(0,0,0), Vector3(0,0,0))
				lstate = LinkState(joint_name,pose,twist,ref_joint_name)

				rospy.wait_for_service('gazebo/set_link_state')
				try:
					set_state = rospy.ServiceProxy('gazebo/set_link_state', SetLinkState)

					resp = set_state.call(SetLinkStateRequest(lstate))


				except rospy.ServiceException, e:
					print "Service call failed: %s"%e

		print "%s -- set state!"%model_name
	##############################################################################################################

	def state_getter(self, data):
		self.link_state = data
		model_name = '%s_%d' % (self.model_name, self.dup_num)
		vel_list = [0,0,0,0,0,0,0,0,0,0]

		for key in list(self.link_reference.keys()):
			link_name = key
			reference_name = self.link_reference[key]
			joint_name = '%s::%s'%(model_name,link_name)
			ref_joint_name = '%s::%s'%(model_name,reference_name)

			if joint_name in data.name:
				twist = data.twist[data.name.index(joint_name)].angular
				ref_twist = data.twist[data.name.index(ref_joint_name)].angular

				vel = math.sqrt((twist.x-ref_twist.x)**2+(twist.y-ref_twist.y)**2+(twist.z-ref_twist.z)**2)
				vel_list[int(key[3])] = vel

		self.joint_states = vel_list
		


	def efforts_caller(self, joint_efforts, duration):
		for i in range(len(joint_efforts)):
			apply_joint_effort('%s_%d::%d'%(self.model_name,self.dup_num,i), joint_efforts[i], time(0,0), duration)

	###############################################################################################################
	def get_pose(self):
		rospy.wait_for_service('gazebo/get_link_state')
		try:
			# create a handle to the add_two_ints service
			get_model_state = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)

			model_name = '%s_%d' % (self.model_name, self.dup_num)
			# formal style
			resp = get_model_state.call(GetLinkStateRequest('%s::CORE'%model_name, ''))

			pos = resp.link_state.pose.position

			return pos

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	##############################################################################################################
	def string_decoder(self, g_str):
		g_str = str(g_str.data).split('/')
		if len(g_str) > (self.dup_num-1):
			return g_str[self.dup_num-1]
		else:
			return -1

	def call_simulate(self, data):
		if data.data[0] == '-':
			return

		local_dir = os.path.dirname(__file__)
		config_file = os.path.join(local_dir, 'config-feedforward')

		config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
							neat.DefaultSpeciesSet, neat.DefaultStagnation,
							config_file)

		gene_dir = os.getenv("WALKYTO_PATH")
		gene_id = self.string_decoder(data)
		if gene_id == -1:
			return

		gene_f = open('%s/src/genes/%s' % (gene_dir, gene_id))
		genome = pickle.load(gene_f)

		net = neat.nn.FeedForwardNetwork.create(genome, config)
		#nn.recurrent.RecurrentNetwork ************** param3
		#-------------------------------------------------------------------------------------------------
		now = rospy.Time.now()

		duration = rospy.Duration(180)
		then = now + duration		

		#self.gazebo_init()
		pos_init = self.get_pose()
		#--------------------------------------------------------------------------------------------------
		dur = rospy.Duration(0.3); gap = rospy.Duration(0)
		while(then > now):
		 	joint_efforts = net.activate(self.joint_states)
		 	self.efforts_caller(joint_efforts, dur)#0.05sec
		 	rospy.sleep(dur)
		 	# print "input:", self.joint_states
		 	# print "output:", joint_efforts
		 	now = rospy.Time.now()
		 	
			if gap == rospy.Duration(0):
				if then-now > duration + rospy.Duration(5):
					gap = then-now
			elif gap > rospy.Duration(0):
				now = now + gap - duration


		 	#print self.dup_num, gap.to_sec(), then.to_sec(), now.to_sec()
		 	
		#--------------------------------------------------------------------------------------------------	
		pos_end = self.get_pose()

		# self.world_clear()
		# if self.init_state != None:
		# 	print len(self.init_state.name)
		# 	self.set_world_state(self.init_state)
		#self.gazebo_exit()

		dist = (pos_end.x - pos_init.x)

		self.fitness = dist
###############################################################################
	
	def fit_server(self, req):
		if self.fitness == None:
			return SimRunResponse(self.fitness, False)
		else:
			fitness = self.fitness
			self.fitness = None
			return SimRunResponse(fitness, True)

	def __init__(self, model_name, dup_t):
		self.total_dup = int(dup_t)
		
		self.model_name = model_name[:-len(model_name.split('_')[-1])-1]
		self.dup_num = int(model_name.split('_')[-1])
		self.joint_states = None
		self.fitness = None		
		self.init_state=None


		if self.model_name == 'MS_Faraday_imu':
			self.link_reference = {'ML-0':'CORE','ML-1':'L2',	'ML-2':'L3-1','ML-3':'L3-1',
							'ML-4':'L4','MR-5':'CORE','MR-6':'R2','MR-7':'R3-1',
							'MR-8':'R3-1','MR-9':'R4'}
		else:
			print 'there is not link_reference of %s' % self.model_name
			sys.exit()

		# self.gazebo_exit()
		rospy.init_node('simulator%d' % self.dup_num)
		

		
		rospy.Subscriber('/gazebo/link_states', LinkStates, self.state_getter)

		# spin() keeps Python from exiting until node is shutdown
		self.gazebo_exit()
		self.gazebo_init()

		rospy.Subscriber('gene_pub', String, self.call_simulate)
		s=rospy.Service('sim_run%d' % self.dup_num, SimRun, self.fit_server)

		rate1=rospy.Rate(2)
		for i in range(20):
			rate1.sleep()
		self.init_state = self.link_state
		
		while(len(self.init_state.name) < self.dup_num*20):
			self.init_state = self.link_state

		
		rospy.spin()


if __name__ == '__main__':
	argv = rospy.myargv()
	model_name = argv[1]
	dup_num = argv[2]

	sim = simulator('%s'%model_name, dup_num)
