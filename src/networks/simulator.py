#!/usr/bin/env python
import neat, visualize
import os, pickle, sys
import rospy

from walkyto.srv import *

def simulate(req):
	local_dir = os.path.dirname(__file__)
	config_file = os.path.join(local_dir, 'config-feedforward')

	config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
						neat.DefaultSpeciesSet, neat.DefaultStagnation,
						config_file)

	# while(True):
	gene_id = req.gene_id
	gene_f = open('./genes/%s' % gene_id)
	genome = pickle.load(gene_f)

	net = neat.nn.FeedForwardNetwork.create(genome, config)
	print gene_id
		# gazebo_init()

		# while(time < 60):
		# 	joint_states = state_getter()
		# 	joint_efforts = net.activate(joint_states)
		# 	efforts_caller(joint_efforts)

		# gazebo_exit()
	return SimRunResponse(100)

def with_generator_server():
	rospy.init_node('simulator', anonymous=True)	
	s = rospy.Service('sim_run', SimRun, simulate)

	# spin() keeps Python from exiting until node is shutdown
	rospy.spin()

if __name__ == '__main__':
	with_generator_server()