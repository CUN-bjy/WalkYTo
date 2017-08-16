#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates
import math

def callback(data):
    # print(data.name)
    # print(data.pose[])
    joint_name_list = ['MS_Faraday_imu::ML-0','MS_Faraday_imu::ML-1','MS_Faraday_imu::ML-2','MS_Faraday_imu::ML-3','MS_Faraday_imu::ML-4','MS_Faraday_imu::MR-5','MS_Faraday_imu::MR-6','MS_Faraday_imu::MR-7','MS_Faraday_imu::MR-8','MS_Faraday_imu::MR-9']

    joint_velocity_list = []

    a= data.twist[data.name.index('MS_Faraday_imu::ML-0')].angular
    b = math.sqrt(a.x**2+a.y**2+a.z**2)
    print(b)

    #for joint_name in joint_name_list:



    # print(data.twist[data.name.index('MS_Faraday_imu::ML-1')])
    # print(data.twist[data.name.index('MS_Faraday_imu::ML-2')])
    # print(data.twist[data.name.index('MS_Faraday_imu::ML-3')])
    # print(data.twist[data.name.index('MS_Faraday_imu::ML-4')])
    # print(data.twist[data.name.index('MS_Faraday_imu::MR-5')])
    # print(data.twist[data.name.index('MS_Faraday_imu::MR-6')])
    # print(data.twist[data.name.index('MS_Faraday_imu::MR-7')])
    # print(data.twist[data.name.index('MS_Faraday_imu::MR-8')])
    # print(data.twist[data.name.index('MS_Faraday_imu::MR-9')])



def listener():
	rospy.init_node('link_states',anonymous = True)
	# gazebo_msgs/LinkStates --> rostopic type
	rospy.Subscriber('/gazebo/link_states', LinkStates, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()
