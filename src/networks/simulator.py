import neat
import os, pickle, sys

def simulate(config_file):
	config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
						neat.DefaultSpeciesSet, neat.DefaultStagnation,
						config_file)

	gene_id = gene_id_getter()
	gene_f = open('./genes/%s' % gene_id)
	genome = pickle.load(gene_f)

	net = neat.nn.FeedForwardNetwork.create(genome, config)
	while(time < 60):
		joint_states = state_getter()
		joint_efforts = net.activate(joint_states)
		efforts_caller(joint_efforts)


if __name__ == '__main__':
	local_dir = os.path.dirname(__file__)
	config_path = os.path.join(local_dir, 'config-feedforward')

	simulate(config_path)