import sys
import os

import rospy
from gazebo_msgs.srv import *

def delete_model(model_name):
    rospy.wait_for_service('gazebo/delete_model')
    
    try:
        # create a handle to the add_two_ints service
        delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        
        # simplified style
        resp1 = delete_model(model_name)
        # print(resp1.success)
        print(resp1.status_message)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# if __name__ == "__main__":
# 	model_name1 = 'MR_bell_imu'
# 	delete_model(model_name1)
