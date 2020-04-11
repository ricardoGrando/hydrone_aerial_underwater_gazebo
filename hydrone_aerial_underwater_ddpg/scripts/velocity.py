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

trans = Transform()
cmd_vel = Twist()

pub = rospy.Publisher('/hydrone_aerial_underwater/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

def velocity_callback(data):
    global cmd_vel
    cmd_vel = data

def position_callback(data):
    #print(data.pose.pose.position.x)
    trans.translation.x = data.pose.pose.position.x
    trans.translation.y = data.pose.pose.position.y
    trans.translation.z = data.pose.pose.position.z
    trans.rotation.x = data.pose.pose.orientation.x
    trans.rotation.y = data.pose.pose.orientation.y
    trans.rotation.z = data.pose.pose.orientation.z
    trans.rotation.w = data.pose.pose.orientation.w

    # cmd_vel.linear.z = 0.1
    #cmd_vel.linear.x = 0.0

    #cmd_vel.angular.z = 0.5
    
    point = MultiDOFJointTrajectoryPoint()
    velocity = MultiDOFJointTrajectory()

    point.transforms.append(trans)
    point.velocities.append(cmd_vel)
    velocity.points.append(point)

    pub.publish(velocity)    

if __name__ == "__main__": 
    rospy.init_node("mission_planner_node", anonymous=False)    

    rospy.Subscriber("/hydrone_aerial_underwater/ground_truth/odometry", Odometry, position_callback)
    rospy.Subscriber("/hydrone_aerial_underwater/cmd_vel", Twist, velocity_callback)

    rospy.spin()