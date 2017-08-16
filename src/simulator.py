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
	def gazebo_init(self, pos):
		local_dir = os.path.dirname(__file__); 
		sdf_file =  open('%s/models/%s/model.sdf'%(local_dir[0:-3],self.model_name), 'r')
		model_xml = sdf_file.read()
		robot_namespace = self.model_name

		orient = orientation(0, 0, 0, 0)
		initial_pose = pose(pos, orient)
		reference_frame=''

		spawn_model(self.model_name, model_xml, robot_namespace, initial_pose, reference_frame)

	def gazebo_exit(self):
		delete_model(self.model_name)
	##############################################################################################################

	def state_getter(self, data):
		link_reference = {'ML-0':'CORE','ML-1':'L2',	'ML-2':'L3-1','ML-3':'L3-1',
						'ML-4':'L4','MR-5':'CORE','MR-6':'R2','MR-7':'R3-1',
						'MR-8':'R3-1','MR-9':'R4'}
		model_name = self.model_name
		vel_list = [0,0,0,0,0,0,0,0,0,0]

		for key in list(link_reference.keys()):
			link_name = key
			reference_name = link_reference[key]
			joint_name = '%s::%s'%(model_name,link_name)
			ref_joint_name = '%s::%s'%(model_name,reference_name)

			if len(data.name) > 1:
				twist = data.twist[data.name.index(joint_name)].angular
				ref_twist = data.twist[data.name.index(ref_joint_name)].angular

				vel = math.sqrt((twist.x-ref_twist.x)**2+(twist.y-ref_twist.y)**2+(twist.z-ref_twist.z)**2)
				vel_list[int(key[3])] = vel

		self.joint_states = vel_list


	def efforts_caller(self, joint_efforts, duration):
		for i in range(len(joint_efforts)):
			apply_joint_effort('%s::%d'%(self.model_name,i), joint_efforts[i], time(0,0), duration)

	###############################################################################################################
	def get_pose(self):
		rospy.wait_for_service('gazebo/get_model_state')
		try:
			# create a handle to the add_two_ints service
			get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

			# formal style
			resp = get_model_state.call(GetModelStateRequest(self.model_name, ''))

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
		now = rospy.Time.now();	duration = rospy.Duration(5)
		then = now + duration
		
		pos = position(0, 0, 0)
		self.gazebo_init(pos)
		pos_init = self.get_pose()
		#--------------------------------------------------------------------------------------------------
		while(then > now):
		 	joint_efforts = net.activate(self.joint_states)
		 	self.efforts_caller(joint_efforts, time(0,100000000))#0.1sec

		 	print "input:", self.joint_states
		 	print "output:", joint_efforts
		 	now = rospy.Time.now()
		#--------------------------------------------------------------------------------------------------
		self.gazebo_exit()
		pos_end = self.get_pose()

		dist = math.sqrt((pos_init.x-pos_end.x)**2+(pos_init.y-pos_end.y)**2+(pos_init.z-pos_end.z)**2)

		return SimRunResponse(dist)


	def __init__(self, model_name):
		self.model_name = model_name
		self.joint_states = ''

		rospy.init_node('simulator')	
		s = rospy.Service('sim_run', SimRun, self.call_simulate)

		rospy.Subscriber('/gazebo/link_states', LinkStates, self.state_getter)

		# spin() keeps Python from exiting until node is shutdown
		rospy.spin()


if __name__ == '__main__':
	sim = simulator('MS_Faraday_imu')