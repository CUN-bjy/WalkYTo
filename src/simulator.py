#!/usr/bin/env python
import neat, visualize
import os, pickle, sys
import rospy
import math, random

from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from walkyto.srv import *

from apply_joint_effort_client import *
from spawn_model_client import *
from delete_model_client import *


class simulator:
	def gazebo_init(self):
		local_dir = os.getenv("GAZEBO_MODEL_PATH")
		sdf_file =  open('%s/%s/model.sdf'%(local_dir, self.model_name), 'r')
		model_xml = sdf_file.read()
		model_name = '%s_%d' % (self.model_name, self.dup_num)
		robot_namespace = model_name

		pos = position(0, (self.dup_num-1)*10, 0)
		orient = orientation(0, 0, 0, 0)
		initial_pose = pose(pos, orient)
		reference_frame=''

		spawn_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)

	def gazebo_exit(self):
		delete_model('%s_%d' % (self.model_name, self.dup_num))
	##############################################################################################################

	def state_getter(self, data):
		if self.model_name == 'MS_Faraday_imu':
			link_reference = {'ML-0':'CORE','ML-1':'L2',	'ML-2':'L3-1','ML-3':'L3-1',
							'ML-4':'L4','MR-5':'CORE','MR-6':'R2','MR-7':'R3-1',
							'MR-8':'R3-1','MR-9':'R4'}
		else:
			print 'there is not link_reference of %s' % self.model_name
			sys.exit()

		model_name = '%s_%d' % (self.model_name, self.dup_num)
		vel_list = [0,0,0,0,0,0,0,0,0,0]

		for key in list(link_reference.keys()):
			link_name = key
			reference_name = link_reference[key]
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
		rospy.wait_for_service('gazebo/get_model_state')
		try:
			# create a handle to the add_two_ints service
			get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

			model_name = '%s_%d' % (self.model_name, self.dup_num)
			# formal style
			resp = get_model_state.call(GetModelStateRequest(model_name, ''))

			pos = resp.pose.position
			return pos

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	##############################################################################################################

	def call_simulate(self, req):
		local_dir = os.path.dirname(__file__)
		config_file = os.path.join(local_dir, 'config-feedforward')

		config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
							neat.DefaultSpeciesSet, neat.DefaultStagnation,
							config_file)

		gene_id = req.gene_id
		gene_f = open('./genes/%s' % gene_id)
		genome = pickle.load(gene_f)

		net = neat.nn.FeedForwardNetwork.create(genome, config)

		#-------------------------------------------------------------------------------------------------
		now = rospy.Time.now();	duration = rospy.Duration(8)
		then = now + duration
		
		self.gazebo_init()
		pos_init = self.get_pose()
		#--------------------------------------------------------------------------------------------------
		while(then > now):
		 	joint_efforts = net.activate(self.joint_states)
		 	self.efforts_caller(joint_efforts, time(0,400000000))#0.4sec

		 	print "input:", self.joint_states
		 	print "output:", joint_efforts
		 	now = rospy.Time.now()
		#--------------------------------------------------------------------------------------------------
		pos_end = self.get_pose()
		self.gazebo_exit()

		dist = math.sqrt((pos_init.x-pos_end.x)**2+(pos_init.y-pos_end.y)**2+(pos_init.z-pos_end.z)**2)

		return SimRunResponse(dist)


	def __init__(self, model_name, dup_t):
		self.total_dup = dup_t
		self.model_name = model_name[0:-2]
		self.dup_num = int(model_name[-1])
		self.joint_states = ''

		rospy.init_node('simulator%d' % self.dup_num)	
		s = rospy.Service('sim_run%d' % self.dup_num, SimRun, self.call_simulate)

		rospy.Subscriber('/gazebo/link_states', LinkStates, self.state_getter)

		# spin() keeps Python from exiting until node is shutdown
		rospy.spin()


if __name__ == '__main__':
	argv = rospy.myargv()
	model_name = argv[1]
	dup_num = argv[2]

	sim = simulator('%s'%model_name, dup_num)

