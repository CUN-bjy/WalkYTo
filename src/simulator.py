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
	def gazebo_init(pos):
		local_dir = os.path.dirname(__file__)
		sdf_file =  open('../models/%s/model.sdf'%self.model_name, 'r')
		model_xml = sdf_file.read()
		robot_namespace = model_name

		orient = orientation(0, 0, 0, 0)
		initial_pose = pose(pos, orient)
		reference_frame=''

		spawn_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)

	def gazebo_exit():
		delete_model(self.model_name)
	##############################################################################################################

	def state_getter(data):
		joint_name = '%s::ML-0'%self.model_name
		a = data.twist[data.name.index(joint_name)].angular

   		self.joint_states = [random.randrange(20) for i in range(10)].append(math.sqrt(a.x**2+a.y**2+a.z**2))


	def efforts_caller(joint_efforts, duration):
		for i in range(len(joint_efforts)):
			apply_joint_effort('%s::%d'%(self.model_name,i), joint_efforts[i], time(0,0), duration)

	###############################################################################################################
	def get_pose():
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

	def call_simulate(req):
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
		gazebo_init(pos)
		pos_init = get_pose()
		#--------------------------------------------------------------------------------------------------
		while(then > now):
		 	joint_efforts = net.activate(self.joint_states)
		 	efforts_caller(joint_efforts, time(0,100000000))#0.1sec

		 	print self.joint_states
		 	print joint_efforts
		 	now = rospy.Time.now()
		#--------------------------------------------------------------------------------------------------
		gazebo_exit()
		pos_end = get_pose()

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