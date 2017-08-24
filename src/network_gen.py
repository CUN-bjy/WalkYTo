#!/usr/bin/env python
import os, pickle
import rospy
import neat, visualize

from walkyto.srv import *
from std_msgs.msg import String


local_dir = os.getenv("WALKYTO_PATH")
if not os.path.exists("%s/src/genes"%local_dir):
		os.makedirs("%s/src/genes"%local_dir, 0766)

def gene_management(genomes):
	local_dir = os.getenv("WALKYTO_PATH")

	# genome_list = []
	# genes_list = os.listdir("%s/src/genes"%local_dir)
	# for genome_id, genome in genomes:
	# 	genome_list.append(str(genome.key))

	# s1 = set(genes_list); s2 = set(genome_list)
	# add_list = [x for x in genome_list if x not in s1]
	# rm_list = [x for x in genes_list if x not in s2]

	# for rm_file in rm_list:
	# 	os.remove("%s/src/genes/%s" % (local_dir,rm_file))

	for genome_id, genome in genomes:
	# 	if(str(genome_id) in add_list):
		gen_file = open("%s/src/genes/%s" % (local_dir,genome_id),'w')
		pickle.dump(genome, gen_file)

def gene_id_publisher(gene_string):
	pub = rospy.Publisher('gene_pub', String, queue_size=10)
	pub.publish(gene_string)

def fit_caller(channel):
	rospy.wait_for_service('sim_run%d'%channel)
	try:
		Sim_Run = rospy.ServiceProxy('sim_run%d' % channel, SimRun, persistent=True)

		resp = Sim_Run.call(SimRunRequest(True))

		if resp.success:
			return resp.distance
		else:
			return -1

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def eval_genomes(genomes, config):
	gene_management(genomes)
	id_list = []; gene_list = []
	for gene_id, gene in genomes:
		id_list.append(gene_id)
		gene_list.append(gene)

	dup_num = 4
	population = 80

	rate1=rospy.Rate(3)# 10hz
	rate2=rospy.Rate(5)# 5hz
	u = 0
	while(dup_num*(u+1)<=population):
		print("-----------simulation(%d/%d)--------------"%(dup_num*(u+1), population))

		gene_string = str(id_list[u*dup_num])
		for i in range(1,dup_num):
			gene_string = gene_string + '/' + str(id_list[u*dup_num+i])

		print("gene_id:", gene_string)
		gene_id_publisher(gene_string)
		rate1.sleep()
		

		fcnt = 0; 
		fit_list=[]
		while fcnt < dup_num:
			gene_id_publisher('-%d'%fcnt)
			fitness = fit_caller(fcnt+1)
			if fitness != -1:
				fit_list.append(fitness)
				gene_list[u*dup_num+fcnt].fitness = fitness
				fcnt = fcnt + 1
			else:
				rate2.sleep()

		
		print("fit:", fit_list)

		u = u+1



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
    p.add_reporter(neat.Checkpointer(1))

    # Run for up to 300 generations.
    winner = p.run(eval_genomes, 150)

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
	# rospy.init_node('network_gen', anonymous=True)	
	rospy.init_node('network_gen')
	
	rate1=rospy.Rate(3)# 10hz
	for i in range(20):
		gene_id_publisher('-');rate1.sleep()

	local_dir = os.path.dirname(__file__)
	config_path = os.path.join(local_dir, 'config-feedforward')
	run(config_path)