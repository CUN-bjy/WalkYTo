#!/usr/bin/env python
import neat, visualize
import os, pickle, sys
import rospy
import math

from gazebo_msgs.srv import *
from walkyto.srv import *

def get_dist(model):
	rospy.wait_for_service('gazebo/get_model_state')
	try:
		# create a handle to the add_two_ints service
		get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

		# formal style
		resp = get_model_state.call(GetModelStateRequest(model, ''))

		pos = resp.pose.position
		return math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


def simulate(req):
	local_dir = os.path.dirname(__file__)
	config_file = os.path.join(local_dir, 'config-feedforward')

	config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
						neat.DefaultSpeciesSet, neat.DefaultStagnation,
						config_file)

	gene_id = req.gene_id
	gene_f = open('./genes/%s' % gene_id)
	genome = pickle.load(gene_f)

	net = neat.nn.FeedForwardNetwork.create(genome, config)

	now = rospy.Time.now();	duration = rospy.Duration(5)
	then = now + duration
	# gazebo_init()

	time = rospy.Time.now()
	while(then < time):
		time = rospy.Time.now()
		print then-time
	# 	joint_states = state_getter()
	# 	joint_efforts = net.activate(joint_states)
	# 	efforts_caller(joint_efforts)
	# gazebo_exit()
	dist = get_dist('MS_Faraday_imu')

	return SimRunResponse(dist)

def with_generator_server():
	rospy.init_node('simulator')	
	s = rospy.Service('sim_run', SimRun, simulate)

	# spin() keeps Python from exiting until node is shutdown
	rospy.spin()

if __name__ == '__main__':
	with_generator_server()