import sys, os
import math

import rospy
from gazebo_msgs.srv import *

def get_link_state(link_name, reference_frame):
    rospy.wait_for_service('gazebo/get_link_state')
    
    try:
        # create a handle to the add_two_ints service
        link_state = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState,persistent=True)
        
        # simplified style
        resp1 = link_state(link_name, reference_frame)
        print resp1.status_message
        return resp1.link_state.twist.angular

        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    link_name1 = 'MR_bell_imu_1::MC-9'  
    reference_frame1 = 'MR_bell_imu_1::C3'
    a = get_link_state(link_name1, reference_frame1)
    b = get_link_state('MR_bell_imu_1::MA-2', 'MR_bell_imu_1::A2')
    # b = get_link_state(link_name2, reference_frame2)
    print(math.sqrt(a.x**2+a.y**2+a.z**2))
    print(math.sqrt(b.x**2+b.y**2+b.z**2))
    # print(b)
    # print(a.x - b.x)
    # print(a.y - b.y)
    # print(a.z - b.z)
