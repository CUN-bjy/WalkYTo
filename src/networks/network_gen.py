""" WalkYTo:Walk Yourself, Toddler!"""
import os
import neat
import visualize

def eval_genomes(genomes, config):
    for genome_id, genome in genomes:
		# if not os.path.exists("./gens"):
		#     os.makedirs("./gens", 0755)

		# gen_file = open('./gens/%d' % genome_id ,'w')
		# pickle.dump(genome, gen_file)
	    	net = neat.nn.FeedForwardNetwork.create(genome, config)
		while(time<60):
			joint_states = state_getter()
			joint_efforts = net.activate(joint_states)
			efforts_caller(joint_efforts)

	genome.fitness = fitness(imu_reader())


def run(config_file):
    # Load configuration.
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_file)

    # Create the population, which is the top-level object for a NEAT run.
    p = neat.Population(config)

    # Add a stdout reporter to show progress in the terminal.
    p.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    p.add_reporter(stats)
    p.add_reporter(neat.Checkpointer(5))

    # Run for up to 300 generations.
    winner = p.run(eval_genomes, 300)

    # Display the winning genome.
    print('\nBest genome:\n{!s}'.format(winner))

    # Show output of the most fit genome against training data.
    print('\nOutput:')
    winner_net = neat.nn.FeedForwardNetwork.create(winner, config)


    node_names = {-1:'A', -2: 'B', 0:'A XOR B'}
    visualize.draw_net(config, winner, True, node_names=node_names)
    visualize.plot_stats(stats, ylog=False, view=True)
    visualize.plot_species(stats, view=True)




if __name__ == '__main__':
    # Determine path to configuration file. This path manipulation is
    # here so that the script will run successfully regardless of the
    # current working directory.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config-feedforward')
    run(config_path)