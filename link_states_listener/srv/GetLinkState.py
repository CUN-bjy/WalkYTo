import sys
import os

import rospy
from gazebo_msgs.srv import *

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
    link_name1 = 'ML-1'
    reference_frame1 = 'CORE'
    link_name2 = 'L2'
    reference_frame2 = 'CORE'
    a = get_link_state(link_name1, reference_frame1)
    b = get_link_state(link_name2, reference_frame2)
    print(a)
    print(b)
    print(a.x - b.x)
    print(a.y - b.y)
    print(a.z - b.z)
