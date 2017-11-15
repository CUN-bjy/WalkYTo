import neat, visualize
import pickle,sys, os
from simul import *

def manual():
	print("\n"),
	print("=============================")
	print("  Analysis tool for WalkYTo")
	print("1. network visualizer")
	print("2. motion tester")
	print("3. I/O viz")
	print("=============================")
	return input("select a work you want>")


def extract_gene():
	filename = input("Drag the stats file and press the enter\n(to quit, enter 'q' including quotation)\n:")
	if filename =="q":
		sys.exit(1)

	stat_file = open(filename)
	stats = pickle.load(stat_file)



	local_dir = os.path.dirname(filename)
	config_file = os.path.join(local_dir, 'config-feedforward')
	# Load configuration
	config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
						neat.DefaultSpeciesSet, neat.DefaultStagnation,
						config_file)



	most_gene = stats.best_unique_genomes(5)
	return most_gene, config
#================================================================================================
def network_visualizer():
	while True:
		print("\n"),
		most_gene, config = extract_gene()

		node_names = {-1:'ML-0(I)', -2:'ML-1(I)', -3:'ML-2(I)', -4:'ML-3(I)',-5:'ML-4(I)',
						-6:'MR-5(I)',-7:'MR-6(I)',-8:'MR-7(I)',-9:'MR-8(I)',-10:'MR-9(I)',
						0:'ML-0(O)', 1:'ML-1(O)', 2:'ML-2(O)', 3:'ML-3(O)',4:'ML-4(O)',5:'MR-5(O)',
						6:'MR-6(O)',7:'MR-7(O)', 8:'MR-8(O)',9:'MR-9(O)'}

		for i in range(5):
			if i != 4:
				view = False
			else:
				view = True
			visualize.draw_net(config, most_gene[i], view=view, node_names=node_names,filename="%d"%i)

			print ("%d:"%i, most_gene[i].fitness)


def motion_tester():
	gazebo_pid = os.fork(); simulator_pid = os.fork()
	if gazebo_pid == 0:
		os.execvp("roslaunch", ('roslaunch', 'walkyto', 'gazebo_empty'))
		sys.exit()
	elif simulator_pid == 0:
		os.execvp("roslaunch", ('roslaunch','walkyto','motion_test.launch'))
		sys.exit()
	else:
		while True:
			most_gene, config = extract_gene()
			gene_num = input('which genome?\n\
								0: gene_0\n\
								1: gene_1\n\
								2: gene_2\n\
								3: gene_3\n\
								4: gene_4\n\
								5: all_gene\n:')
			if gene_num != 5:
				for i in range(5):
					gen_file = open("%s/src/genes/%d" % (getenv(WALKYTO_PATH),i),'w')
					pickle.dump(most_gene[i], gen_file)

				print("this function does not work now. not completed!")
				return
			else:
				gen_file = open("%s/src/genes/%d" % (getenv(WALKYTO_PATH),gene_num),'w')
				pickle.dump(most_gene[gene_num], gen_file)


#==========================================================================================
if __name__ == '__main__':
	cur_child_cnt = 0

	while True:
		selected = manual()

		if selected == 1:
			pid = os.fork()
			if pid == 0:
				network_visualizer()
				sys.exit()
			cur_child_cnt += 1

		elif selected == 2:
			pid = os.fork()
			if pid == 0:
				motion_tester()
				sys.exit()
			cur_child_cnt += 1

		else:
			print("\n!!wrong typed!!")

		if cur_child_cnt > 0:
			os.wait()
			cur_child_cnt-=1