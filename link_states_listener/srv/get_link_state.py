import sys
import os

import rospy
from gazebo_msgs.srv import *
import math

def get_link_state(link_name, reference_frame):
    rospy.wait_for_service('gazebo/get_link_state')
    
    try:
        # create a handle to the add_two_ints service
        link_state = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)
        
        # simplified style
        resp1 = link_state(link_name, '')
        return resp1.link_state.twist.angular
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    link_name1 = 'MS_Faraday_imu::L1'
    link_name2 = 'MS_Faraday_imu::ML-1'
   
    a = get_link_state(link_name1, '')
    b = get_link_state(link_name2, '')
    print(a.x - b.x)
    print(a.y - b.y)
    print(a.z - b.z)
    print((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)
    # print(math.sqrt(a.x**2 + a.y**2 + a.z**2))
    #print(math.sqrt(b.x**2 + b.y**2 + b.z**2))
    
