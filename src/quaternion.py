import rospy, math, numpy as np
from gazebo_msgs.msg import LinkStates

link_reference = {'ML-0':'CORE','ML-1':'L2',
'ML-2':'L3-1','ML-3':'L3-1','ML-4':'L4','MR-5':'CORE',
'MR-6':'R2','MR-7':'R3-1','MR-8':'R3-1','MR-9':'R4'}

def dotted_func(self, data):
    model_name = '%s_%d' % (self.model_name, self.dup_num)
    position_list = [0,0,0,0,0,0,0,0,0,0]
    link_position = []; M_position = []; rel_pos = []

    for key in ['L1','L5','R1','R5']:
        joint_name = '%s::%s'%(model_name,key)
        if joint_name in data.name:
            link_position.append(data.pose[data.name.index(joint_name)].position)

    for M_name in ['ML-1','ML-4','MR-6','MR-9']:
        joint_name = '%s::%s'%(model_name,M_name)
        if joint_name in data.name:
            M_position.append(data.pose[data.name.index(joint_name)].position)

    if len(link_position)==4 and len(M_position)==4:
        for i in range(4):
            rel_pos.append([link_position[i].x - M_position[i].x,link_position[i].y - M_position[i].y,link_position[i].z - M_position[i].z])
    else:
        # print len(link_position),len(M_position)
        return [0,0,0,0,0,0,0,0,0,0]

    for key in list(self.link_reference.keys()):
        link_name = key
        reference_name = self.link_reference[key]
        joint_name = '%s::%s'%(model_name,link_name)
        ref_joint_name = '%s::%s'%(model_name,reference_name)

        # orientation = data.pose[data.name.index(joint_name)].orientation
        # ref_orientation = data.pose[data.name.index(ref_joint_name)].orientation
        if joint_name in data.name:
            position = data.pose[data.name.index(joint_name)].position
            ref_position = data.pose[data.name.index(ref_joint_name)].position
            relative_pos = np.array([position.x - ref_position.x,position.y - ref_position.y,position.z - ref_position.z])
            # print(np.linalg.norm(relative_pos))
            # print(relative_pos)

            if key[3] == '0':
                core_vec_L = relative_pos

                L3_vec = data.pose[data.name.index('%s::L3-1'%(model_name))].position
                ML0_vec = [L3_vec.x - position.x,L3_vec.y - position.y,L3_vec.z - position.z]
                position_list[int(key[3])] = np.array(ML0_vec)

            elif key[3] == '5':
                core_vec_R = relative_pos

                R3_vec = data.pose[data.name.index('%s::R3-1'%(model_name))].position
                ML5_vec = [R3_vec.x - position.x,R3_vec.y - position.y,R3_vec.z - position.z]
                position_list[int(key[3])] = np.array(ML5_vec)

            else:
                position_list[int(key[3])] = relative_pos   


    position_list.insert(10,rel_pos[3]);position_list.insert(6,rel_pos[2]);position_list.insert(5,rel_pos[1]);position_list.insert(1,rel_pos[0])
    # for i in range(14):
    #     print(np.linalg.norm(position_list[i]))
    cross_vec = [ np.cross(core_vec_L,position_list[0]), 
            np.cross(position_list[1],position_list[2]),
            np.cross(position_list[2],position_list[3]),
            np.cross(position_list[4],position_list[5]),
            np.cross(position_list[5],position_list[6]),
            
            np.cross(core_vec_R,position_list[7]),
            np.cross(position_list[9],position_list[8]),
            np.cross(position_list[10],position_list[9]),
            np.cross(position_list[12],position_list[11]),
            np.cross(position_list[13],position_list[12])]
    # print(cross_vec)
    # print(position_list)
    dotted =[]
    for i in range (10):
        if i ==0:
            val = np.dot(position_list[3], cross_vec[i])
        elif i ==5:
            val = np.dot(position_list[4], cross_vec[i])
        elif i < 5:
            val = np.dot(core_vec_L,cross_vec[i])
        else:
            val = np.dot(core_vec_R,cross_vec[i])
            
        if val==0:
            dotted.append(0)
        else:
            dotted.append(val/abs(val))
            
    return dotted


def call_quat(data):
    model_name = 'MS_Faraday_imu'
    rel_q_list = [0,0,0,0,0,0,0,0,0,0]
    print("======================================")
    for key in list(link_reference.keys()):
        link_name = key
        reference_name = link_reference[key]
        joint_name = '%s::%s'%(model_name,link_name)
        ref_joint_name = '%s::%s'%(model_name,reference_name)

        orientation = data.pose[data.name.index(joint_name)].orientation
        ref_orientation = data.pose[data.name.index(ref_joint_name)].orientation

        q2_vec = np.array([orientation.x,orientation.y,orientation.z])
        q2_0 = orientation.w

        q1_vec = np.array([ref_orientation.x,ref_orientation.y,ref_orientation.z])
        q1_0 = ref_orientation.w

        rel_q_w = q1_0*q2_0  + np.dot(q1_vec, q2_vec)
        rel_q_vec =  - q1_0*q2_vec + q2_0*q1_vec + np.cross(q2_vec,q1_vec)
        # ======================================================== #
        rel_q_list[int(key[3])] = rel_q_w

    dotted = dotted_func(data)
    for i in range(10):
        print(i, "ang :", math.acos(rel_q_list[i])*2*180/3.141592*dotted[i])


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('position listener', anonymous=True)
    rospy.Subscriber('/gazebo/link_states', LinkStates, call_quat)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()