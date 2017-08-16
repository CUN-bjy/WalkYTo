#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates
import math

link_reference = {'ML-0':'CORE','ML-1':'L2',
'ML-2':'L3-1','ML-3':'L3-1','ML-4':'L4','MR-5':'CORE',
'MR-6':'R2','MR-7':'R3-1','MR-8':'R3-1','MR-9':'R4'}

def callback(data):
    # print(data.name)
    # print(data.pose[])
    model_name = 'MS_Faraday_imu'
    vel_list = [0,0,0,0,0,0,0,0,0,0]

    for key in list(link_reference.keys()):
        link_name = key
        reference_name = link_reference[key]
        joint_name = '%s::%s'%(model_name,link_name)
        ref_joint_name = '%s::%s'%(model_name,reference_name)

        twist = data.twist[data.name.index(joint_name)].angular
        ref_twist = data.twist[data.name.index(ref_joint_name)].angular

        vel = math.sqrt((twist.x-ref_twist.x)**2+(twist.y-ref_twist.y)**2+(twist.z-ref_twist.z)**2)
        vel_list[int(key[3])] = vel
    #for joint_name in joint_name_list:
    print(vel_list)


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
