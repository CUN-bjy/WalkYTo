import neat, visualize
import pickle,sys, os, time, signal
import rospy

from std_msgs.msg import String
from std_srvs.srv import Empty
from walkyto.srv import *

from termcolor import cprint

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
		return False,False

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
		if most_gene == False:
			return

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
		c_log = os.open("core_log", os.O_RDWR|os.O_CREAT|os.O_TRUNC)
		os.close(sys.__stdout__.fileno());os.dup(c_log)
		os.close(sys.__stderr__.fileno());os.dup(c_log)

		os.execlp("roscore", 'roscore')
		sys.exit()

	simulator_pid = os.fork()
	if simulator_pid == 0:
		time.sleep(3)
		s_log = os.open("simulator_log", os.O_RDWR|os.O_CREAT|os.O_TRUNC)
		os.close(sys.__stdout__.fileno());os.dup(s_log)
		os.close(sys.__stderr__.fileno());os.dup(s_log)

		os.execvp("roslaunch", ('roslaunch','walkyto','motion_test.launch'))
		sys.exit()

	cprint("boot on the gazebo & simulators(12s)", 'blue', 'on_white')
	for i in range(12):
		time.sleep(1)
		os.write(sys.__stderr__.fileno(),"###")
		if i == 9:
			client_pid = os.fork()
			if client_pid == 0:
				c_log = os.open("client_log", os.O_RDWR|os.O_CREAT|os.O_TRUNC)
				os.close(sys.__stdout__.fileno());os.close(sys.__stderr__.fileno())
				stdout = os.dup(c_log);stderr = os.dup(c_log)

				os.execvp("roslaunch", ('roslaunch', 'walkyto', 'gzclient.launch'))
				sys.exit()
	print("\n"),


	rospy.init_node('analyst')
	while True:
		most_gene, config = extract_gene()
		if most_gene == False:
			break

		while True:
			print("which genome?")

			for i in range(5):
				print("%d) gene_%d(%f)"%(i,i, most_gene[i].fitness))
			gene_num = input("5) all\n6) quit\n\n:")


			if gene_num == 5:
				for i in range(5):
					time.sleep(0.1)
					gene_id_publisher('-')
				gazebo_clear()
				for i in range(5):
					gen_file = open("%s/src/genes/%d" % (os.getenv('WALKYTO_PATH'),i),'w')
					pickle.dump(most_gene[i], gen_file)
					gen_file.close()

				gene_string = '0/1/2/3/4'

				print("gene_id:", gene_string)
				gene_id_publisher(gene_string)

				fcnt = 0; fit_list=[]
				while fcnt < 5:
					gene_id_publisher('-%d'%fcnt)
					fitness = fit_caller(fcnt+1)
					if fitness != -1:
						fit_list.append(fitness)
						fcnt = fcnt + 1
					else:
						now = rospy.get_rostime()
						if now.secs%10 == 0 :
							rospy.loginfo("Current time %i %.3i", now.secs, now.nsecs)
						time.sleep(0.5)

				print("fit:", fit_list)

			elif gene_num==0 or gene_num==1 or gene_num==2 or gene_num==3 or gene_num==4:
				for i in range(5):
					time.sleep(0.1)
					gene_id_publisher('-')
				gazebo_clear()
				gen_file = open("%s/src/genes/%d" % (os.getenv('WALKYTO_PATH'),gene_num),'w')
				pickle.dump(most_gene[gene_num], gen_file)
				gen_file.close()

				gene_string = '%s/-1/-1/-1/-1'%str(gene_num)
				print("gene_id:", str(gene_num))
				gene_id_publisher(gene_string)

				fcnt = 0; fit_list=[]
				while fcnt == 0:
					gene_id_publisher('-%d'%fcnt)
					fitness = fit_caller(fcnt+1)
					if fitness != -1:
						fit_list.append(fitness)
						fcnt = fcnt + 1
					else:
						now = rospy.get_rostime()
						if now.secs%10 == 0 :
							rospy.loginfo("Current time %i %.3i", now.secs, now.nsecs)
						time.sleep(0.5)

				print("fit:", fit_list)
			else:
				break

	os.kill(simulator_pid, signal.SIGINT); os.kill(client_pid, signal.SIGINT)
	cprint("boot down the gazebo & simulators(5s)", 'blue', 'on_white')
	for i in range(5):
		time.sleep(1)
		os.write(sys.__stderr__.fileno(),"#######")
	print("##\n"),


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
		elif selected == 3:
			print("\nit is not completed function")
		else:
			print("\n!!wrong typed!! I'm gonna exit")
			sys.exit()

		if cur_child_cnt > 0:
			os.wait()
			cur_child_cnt-=1