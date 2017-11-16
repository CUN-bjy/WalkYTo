import neat, visualize
import pickle,sys, os, time, signal
import rospy

from std_msgs.msg import String
from walkyto.srv import *


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
#===================================================================================================
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

def gazebo_clear():
	rospy.wait_for_service('gazebo/reset_simulation')
	try:
		rs_sim = rospy.ServiceProxy('gazebo/reset_simulation', Empty)

		resp = rs_sim.call()

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


def motion_tester():
	core_pid = os.fork()
	if core_pid == 0:
		c_log = os.open("core_log", os.O_RDWR|os.O_CREAT)
		os.close(sys.__stdout__.fileno());os.close(sys.__stderr__.fileno())
		os.dup(c_log);os.dup(c_log)
		os.execlp("roscore", 'roscore')
		sys.exit()

	gazebo_pid = os.fork()
	if gazebo_pid == 0:
		time.sleep(4)

		g_log = os.open("gazebo_log", os.O_RDWR|os.O_CREAT)
		os.close(sys.__stdout__.fileno());os.close(sys.__stderr__.fileno())
		os.dup(g_log);os.dup(g_log)

		os.execvp("roslaunch", ('roslaunch', 'walkyto', 'gazebo_empty.launch'))
		sys.exit()

	simulator_pid = os.fork()
	if simulator_pid == 0:
		time.sleep(8)
		s_log = os.open("simulator_log", os.O_RDWR|os.O_CREAT)
		os.close(sys.__stdout__.fileno());os.close(sys.__stderr__.fileno())
		os.dup(s_log);os.dup(s_log)

		os.execvp("roslaunch", ('roslaunch','walkyto','motion_test.launch'))
		sys.exit()

	time.sleep(10)
	while True:
		most_gene, config = extract_gene()
		gene_num = input("which genome?\n0: gene_0\n1: gene_1\n2: gene_2\n3: gene_3\n4: gene_4\n5: all_gene\n6: quit\n:")

		if gene_num != 5:
			for i in range(5):
				gen_file = open("%s/src/genes/%d" % (os.getenv('WALKYTO_PATH'),i),'w')
				pickle.dump(most_gene[i], gen_file)

			gene_string = str(most_gene.pop())
			while len(most_gene) > 0:
				gene_string = gene_string + '/' + str(most_gene.pop())
			

			fcnt = 0; fit_list=[]
			while fcnt < 5:
				gene_id_publisher('-%d'%fcnt)
				fitness = fit_caller(fcnt+1)
				if fitness != -1:
					fit_list.append(fitness)
					fcnt = fcnt + 1
				else:
					time.sleep(0.2)

			gazebo_clear()
			print("fit:", fit_list)

		elif gene_num==0 or gene_num==1 or gene_num==2 or gene_num==3 or gene_num==4:
			gen_file = open("%s/src/genes/%d" % (os.getenv('WALKYTO_PATH'),gene_num),'w')
			pickle.dump(most_gene[gene_num], gen_file)

			gene_string = str(most_gene[gene_num])

			fcnt = 0; fit_list=[]
			while fcnt == 0:
				gene_id_publisher('-%d'%fcnt)
				fitness = fit_caller(fcnt+1)
				if fitness != -1:
					fit_list.append(fitness)
					fcnt = fcnt + 1
				else:
					time.sleep(0.2)

			gazebo_clear()
			print("fit:", fit_list)

		else:
			os.kill(gazebo_pid, signal.SIGINT); os.kill(simulator_pid, signal.SIGINT); os.kill(core_pid, signal.SIGINT)
			return


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