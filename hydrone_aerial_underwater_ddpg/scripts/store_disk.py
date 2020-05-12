#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *


def store_disk(data):
    file_object = open('/home/ricardo/catkin_ws/src/hydrone_aerial_underwater_gazebo/hydrone_aerial_underwater_ddpg/scripts/resultDdpg_paper_env1.csv', 'a')
    file_object.write(data.data+'\n')

if __name__ == "__main__": 
    rospy.init_node("store_disk", anonymous=False)    

    rospy.Subscriber("/result", String, store_disk)

    rospy.spin()