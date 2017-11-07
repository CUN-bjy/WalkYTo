#!/usr/bin/env python
import os, pickle
import rospy
import neat, visualize
import time

from walkyto.srv import *
from std_msgs.msg import String
from std_srvs.srv import Empty


def gazebo_clear():
	rospy.wait_for_service('gazebo/reset_simulation')
	try:
		rs_sim = rospy.ServiceProxy('gazebo/reset_simulation', Empty)

		resp = rs_sim.call()

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def world_clear():
	rospy.wait_for_service('gazebo/reset_world')
	try:
		rs_sim = rospy.ServiceProxy('gazebo/reset_world', Empty)

		resp = rs_sim.call()

		print("world reset!")

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
local_dir = os.getenv("WALKYTO_PATH")
if not os.path.exists("%s/src/genes"%local_dir):
		os.makedirs("%s/src/genes"%local_dir, 0766)

def gene_management(genomes):
	local_dir = os.getenv("WALKYTO_PATH")

	genome_list = []
	genes_list = os.listdir("%s/src/genes"%local_dir)
	for genome_id, genome in genomes:
		genome_list.append(str(genome.key))

	s1 = set(genes_list); s2 = set(genome_list)
	add_list = [x for x in genome_list if x not in s1]
	rm_list = [x for x in genes_list if x not in s2]

	for rm_file in rm_list:
		os.remove("%s/src/genes/%s" % (local_dir,rm_file))

	for genome_id, genome in genomes:
		gen_file = open("%s/src/genes/%d" % (local_dir,genome_id),'w')
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

	dup_num = 8
	population = len(id_list)
	temp_list =list(id_list)

	rate1=rospy.Rate(3)# 10hz
	u = 0
	while(temp_list != []):
		gene_string = str(temp_list.pop())

		for i in range(1,dup_num):
			if len(temp_list) > 0:
				gene_string = gene_string + '/' + str(temp_list.pop())
			else:
				break

		print("-----------simulation(%d/%d)--------------"%(population-len(temp_list), population))
		print("gene_id:", gene_string)
		gene_id_publisher(gene_string)
		# rate1.sleep()
		

		fcnt = 0; fit_list=[]
		while fcnt < dup_num:
			gene_id_publisher('-%d'%fcnt)
			fitness = fit_caller(fcnt+1)
			if fitness != -1:
				fit_list.append(fitness)
				a_gen = gene_list.pop()
				a_gen.fitness = fitness
				fcnt = fcnt + 1
			else:
				time.sleep(0.2)

			if len(gene_list) == 0:
				break

		# world_clear()
		gazebo_clear()
		print("fit:", fit_list)

		u = u+1

	# gazebo_clear()
	# print 'pass'

""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def run(config_file, max_iter):
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
    p.add_reporter(neat.Checkpointer(generation_interval=5,time_interval_seconds=None))

    # Run for up to 300 generations.
    winner = p.run(eval_genomes, n=int(max_iter))


    node_names = {-1:'ML-0(I)', -2:'ML-1(I)', -3:'ML-2(I)', -4:'ML-3(I)',-5:'ML-4(I)',
    			-6:'MR-5(I)',-7:'MR-6(I)',-8:'MR-7(I)',-9:'MR-8(I)',-10:'MR-9(I)',
				0:'ML-0(O)', 1:'ML-1(O)', 2:'ML-2(O)', 3:'ML-3(O)',4:'ML-4(O)',5:'MR-5(O)',
				6:'MR-6(O)',7:'MR-7(O)', 8:'MR-8(O)',9:'MR-9(O)'}
    visualize.draw_net(config, winner, view=True, node_names=node_names)
    visualize.plot_stats(stats, ylog=False, view=True)
    visualize.plot_species(stats, view=True)


def load_checkpoint(ckp_name, max_iter):
	p = neat.Checkpointer.restore_checkpoint(ckp_name)

	p.add_reporter(neat.StdOutReporter(True))
	stats = neat.StatisticsReporter()
	p.add_reporter(stats)
	p.add_reporter(neat.Checkpointer(1))

	p.run(eval_genomes, max_iter)

if __name__ == '__main__':
    # Determine path to configuration file. This path manipulation is
    # here so that the script will run successfully regardless of the
    # current working directory.
	# rospy.init_node('network_gen', anonymous=True)	
	rospy.init_node('network_gen')
	argv = rospy.myargv()
	
	rate1=rospy.Rate(2)
	for i in range(20):
		gene_id_publisher('-');rate1.sleep()

	local_dir = os.path.dirname(__file__)
	config_path = os.path.join(local_dir, 'config-feedforward')
	
	if argv[2] == '-r':
		run(config_path, argv[1])
	elif argv[2] == '-l':
		load_checkpoint(argv[3],argv[1])