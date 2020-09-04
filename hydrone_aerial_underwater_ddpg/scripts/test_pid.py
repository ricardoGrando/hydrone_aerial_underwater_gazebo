#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geographic_msgs.msg import *
from trajectory_msgs.msg import *
from nav_msgs.msg import Odometry
import math

pub = rospy.Publisher('/hydrone_aerial_underwater/command/pose', PoseStamped, queue_size=10)

# posx = [0.0, 2.0, 0.0, -2.0, -2.0, 0.0, 2.0, 0.0]
# posy = [0.0, 2.0, 3.0, 2.0, -2.0, -3.0, -2.0, 0.0]
# posz = [-1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5]

i = 0

# posx = [0.0, 3.6, -3.6, -3.6, 0.0]
# posy = [0.0, 2.6, 3.0, 1.0, 0.0]
# posz = [-1.5, -1.5, -1.5, -1.5, -1.5]

# posx = [0.0, -1.5, 0.0, -1.5, -1.5, 0.0, 1.5, 0.0]
# posy = [0.0, -1.5, 1.5, 1.5, -1.5, -1.5, -1.5, 0.0]
# posz = [2.5, 2.5, 1.0, 2.5, 1.0, 2.5, 1.0, 2.5]

posx = [0.0, 3.6, -3.6, -3.6, 0.0]
posy = [0.0, 2.6, 3.0, 1.0, 0.0]
posz = [2.5, 1.0, 2.5, 1.0, 2.5]

# goal_x_list = [3.6, -3.6, -3.6, 0.0]
# goal_y_list = [2.6, 3.0, 1.0, 0.0]
   
# goal_x_list = [2.0, 0.0, -2.0, -2.0, 0.0, 2.0, 0.0]
# goal_y_list = [2.0, 3.0, 2.0, -2.0, -3.0, -2.0, 0.0]

def state_callback(data):
    global i

    i = data.data

    print(i)

def position_callback(data):
    global posx
    global posy
    global posz
    global i
    global pub

    distance = math.sqrt((posx[i] - data.pose.pose.position.x)**2 + (posy[i] - data.pose.pose.position.y)**2 + (posz[i] - data.pose.pose.position.z)**2)
    
    pose = PoseStamped()
    pose.pose.position.x = posx[i]
    pose.pose.position.y = posy[i]
    pose.pose.position.z = posz[i]

    # print(distance, i)
    if (distance < 0.25):
        i += 1
        print(i)

    pub.publish(pose)
    

if __name__ == "__main__": 
    rospy.init_node("test_pdi", anonymous=False)    

    rospy.Subscriber("/hydrone_aerial_underwater/ground_truth/odometry", Odometry, position_callback)

    rospy.Subscriber("/hydrone_aerial_underwater/next_position_pid", Int64, state_callback)
   
    rospy.spin()
    

    