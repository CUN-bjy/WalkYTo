import os, signal, time
import rospy

pid = os.fork()
if pid == 0:
	os.execvp("roslaunch", ('roslaunch','walkyto', 'test.launch'))
else:
	print( "Im waiting")
	time.sleep(5)
	os.kill(pid, signal.SIGINT)
	print("I'll kill you!")
	os.wait()
	print("done")