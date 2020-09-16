#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

PATH = 'sac_stage_1/sac_env1_2d_3_layers'
ROOT = '/home/ricardo/'

def store_disk(data):
    global NAME
    global ROOT
    file_object = open(ROOT+'catkin_ws/src/hydrone_aerial_underwater_gazebo/hydrone_aerial_underwater_ddpg/scripts/Models/'+PATH+'/'+PATH+'.csv', 'a')
    
    file_object.write(data.data+'\n')

def pose_callback(data):
    # print(data.position.x)
    file_object = open('/home/ricardo/catkin_ws/src/hydrone_aerial_underwater_gazebo/hydrone_aerial_underwater_ddpg/scripts/position_sac_2_air_waypoint_3d.csv', 'a')
    file_object.write(str(data.position.x)+","+str(data.position.y)+","+str(data.position.z)+'\n')
    # time.sleep(0.1)
    # print(data)

if __name__ == "__main__":
    global NAME 
    global ROOT
    rospy.init_node("store_disk", anonymous=False)   

    PATH = rospy.get_param('~file_path') 
    ROOT = rospy.get_param('~root_path') 

    rospy.Subscriber("/result", String, store_disk)

    # rospy.Subscriber("/hydrone_aerial_underwater/odometry_sensor1/pose", Pose, pose_callback)

    rospy.spin()