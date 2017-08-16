import sys
import os

import rospy
from gazebo_msgs.srv import *

class pose :
	def __init__(self, position, orientation):
		self.position = position
		self.orientation = orientation

class position :
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z

class orientation :
	def __init__(self, x, y, z, w):
		self.x = x
		self.y = y
		self.z = z
		self.w = w

def spawn_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame):
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    
    try:
        # create a handle to the add_two_ints service
        spawn_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        
        # simplified style
        resp1 = spawn_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
        # print(resp1.success)
        print(resp1.status_message)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# if __name__ == "__main__":
# 	model_name1 = 'MR_bell_imu'
# 	sdf_file =  open('/home/seunghwanyu/catkin_ws/src/WalkYTo/models/MR_bell_imu/MR_bell_imu_model.sdf','r')
# 	model_xml1 = sdf_file.read()
# 	robot_namespace1 = 'MR_bell_imu'
# 	position1 = position(1,1,0)
# 	orientation1 = orientation(0, 0, 0, 0)
# 	initial_pose1 = initial_pose(position1, orientation1)
# 	reference_frame1 = ''

# 	spawn_model(model_name1, model_xml1, robot_namespace1, initial_pose1, reference_frame1)
