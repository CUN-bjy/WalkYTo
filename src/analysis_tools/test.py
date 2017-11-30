# import os, signal, time
# import rospy

# pid = os.fork()
# if pid == 0:
# 	os.execlp("roscore", 'roscore')
# else:
# 	print( "Im waiting")
# 	time.sleep(5)
# 	os.kill(pid, signal.SIGINT)
# 	print("I'll kill you!")
# 	os.wait()
# 	print("done")

def string_decoder(g_str, dup_num):
	g_str = str(g_str).split('/')
	print g_str
	if len(g_str) > (dup_num-1):
		return g_str[dup_num-1]
	else:
		return -1
for i in range(5):
	print string_decoder(input(":"), i)