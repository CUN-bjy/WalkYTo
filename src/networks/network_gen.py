""" WalkYTo:Walk Yourself, Toddler!"""
import os, pickle
import rospy
import neat, visualize


if not os.path.exists("./genes"):
		os.makedirs("./genes", 0755)

def gene_management(genomes):
	genome_list = []
	genes_list = os.listdir("./genes")
	for genome_id, genome in genomes:
		genome_list.append(str(genome.key))
		genome.fitness = 1

	s1 = set(genes_list); s2 = set(genome_list)
	add_list = [x for x in genome_list if x not in s1]
	rm_list = [x for x in genes_list if x not in s2]

	for rm_file in rm_list:
		os.remove("./genes/%s" % rm_file)

	for genome_id, genome in genomes:
		if(str(genome_id) in add_list):
			gen_file = open('./genes/%d' % genome_id ,'w')
			pickle.dump(genome, gen_file)

	return os.listdir("./genes")


def fitness(sam_num):
	dist = 1
	return dist

def eval_genomes(genomes, config):
	genes = gene_management(genomes)

	while(genes != []):
		sample1 = genes.pop(); sample2 = genes.pop()
		sample3 = genes.pop(); sample4 = genes.pop()
		print(sample1, sample2, sample3, sample4)

		sim(1, sample1); sim(2, sample2); sim(3, sample3); sim(4, sample4)
		wait(60)
		fit1 = fitness(1); fit2 = fitness(2)
		fit3 = fitness(3); fit4 = fitness(4)
		print(fit1, fit2, fit3, fit4)

		for genome_id, genome in genomes:
			if(str(genome_id) == sample1):
				genome.fitness = fit1
			elif(str(genome_id) == sample2):
				genome.fitness = fit2
			elif(str(genome_id) == sample3):
				genome.fitness = fit3
			elif(str(genome_id) == sample4):
				genome.fitness = fit4



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
	rospy.init_node('network_gen', anonymous=True)	

	local_dir = os.path.dirname(__file__)
	config_path = os.path.join(local_dir, 'config-feedforward')
	run(config_path)