#!/usr/bin/env python
import neat, visualize
import os, pickle, sys
import rospy
import math, random

from gazebo_msgs.srv import *
from walkyto.srv import *
###############################################################################################
class time:
	def __init__(self, secs, nsecs):
		self.secs = secs
		self.nsecs = nsecs

def apply_joint_effort(joint_name, effort, start_time, duration ):
    rospy.wait_for_service('gazebo/apply_joint_effort')
    
    try:
        # create a handle to the add_two_ints service
        apply_joint = rospy.ServiceProxy('gazebo/apply_joint_effort', ApplyJointEffort, persistent=True)
        
        # simplified style
        resp1 = apply_joint(joint_name, effort, start_time, duration)
        print(resp1.status_message)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def efforts_caller(Robot, joint_efforts, duration):
	for i in range(len(joint_efforts)):
		apply_joint_effort('%s::%d'%(Robot,i), joint_efforts[i], time(0,0), duration)

###############################################################################################################
def get_pose(model):
	rospy.wait_for_service('gazebo/get_model_state')
	try:
		# create a handle to the add_two_ints service
		get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

		# formal style
		resp = get_model_state.call(GetModelStateRequest(model, ''))

		pos = resp.pose.position
		return pos

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
##############################################################################################################

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
	pos_init = get_pose('MS_Faraday_imu')

	while(then > now):
	# 	joint_states = state_getter()
		joint_states = [random.randrange(20) for i in range(11)]
	 	joint_efforts = net.activate(joint_states)
	 	efforts_caller('MS_Faraday_imu', joint_efforts, time(0,100000000))#0.1sec
	 	print joint_states
	 	print joint_efforts
	 	now = rospy.Time.now()

	# gazebo_exit()
	pos_end = get_pose('MS_Faraday_imu')

	dist = math.sqrt((pos_init.x-pos_end.x)**2+(pos_init.y-pos_end.y)**2+(pos_init.z-pos_end.z)**2)

	return SimRunResponse(dist)


def with_generator_server():
	rospy.init_node('simulator')	
	s = rospy.Service('sim_run', SimRun, simulate)

	# spin() keeps Python from exiting until node is shutdown
	rospy.spin()

if __name__ == '__main__':
	with_generator_server()