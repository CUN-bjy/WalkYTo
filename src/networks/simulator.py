import neat, visualize
import os, pickle, sys
import rospy

def simulate(config_file):
	config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
						neat.DefaultSpeciesSet, neat.DefaultStagnation,
						config_file)

	# while(True):
	gene_id = 4 #gene_id_getter()
	gene_f = open('./genes/%s' % gene_id)
	genome = pickle.load(gene_f)

	net = neat.nn.FeedForwardNetwork.create(genome, config)
		# gazebo_init()

		# while(time < 60):
		# 	joint_states = state_getter()
		# 	joint_efforts = net.activate(joint_states)
		# 	efforts_caller(joint_efforts)

		# gazebo_exit()


if __name__ == '__main__':
	rospy.init_node('simulator', anonymous=True)	

	local_dir = os.path.dirname(__file__)
	config_path = os.path.join(local_dir, 'config-feedforward')

	simulate(config_path)