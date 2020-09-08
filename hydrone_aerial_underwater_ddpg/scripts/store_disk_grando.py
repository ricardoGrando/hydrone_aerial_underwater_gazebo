#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

NAME = 'ddpg_env2_2d'

flag = True

def store_disk(data):
    global NAME
    global flag
    if (flag):
        file_object = open('/content/catkin_ws/src/hydrone_aerial_underwater_gazebo/hydrone_aerial_underwater_ddpg/scripts/Models/ddpg_stage_1/'+NAME+'.csv', 'a')
    else:
        file_object = open('/home/ricardo/catkin_ws/src/hydrone_aerial_underwater_gazebo/hydrone_aerial_underwater_ddpg/scripts/Models/ddpg_stage_1/'+NAME+'.csv', 'a')
    
    file_object.write(data.data+'\n')

def pose_callback(data):
    # print(data.position.x)
    file_object = open('/home/ricardo/catkin_ws/src/hydrone_aerial_underwater_gazebo/hydrone_aerial_underwater_ddpg/scripts/position_sac_2_air_waypoint_3d.csv', 'a')
    file_object.write(str(data.position.x)+","+str(data.position.y)+","+str(data.position.z)+'\n')
    # time.sleep(0.1)
    # print(data)

if __name__ == "__main__": 
    rospy.init_node("store_disk", anonymous=False)    

    rospy.Subscriber("/result", String, store_disk)

    # rospy.Subscriber("/hydrone_aerial_underwater/odometry_sensor1/pose", Pose, pose_callback)

    rospy.spin()