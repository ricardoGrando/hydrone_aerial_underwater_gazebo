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
cmd_vel_lin = 0.0
pi = 3.14159265359

pub = rospy.Publisher('/hydrone_aerial_underwater/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def velocity_callback(data):
    global cmd_vel
    global cmd_vel_lin
    global ang_z

    cmd_vel = data

    cmd_vel_lin = cmd_vel.linear.x

fake_pitch = pi/2

def position_callback(data):
    global cmd_vel
    global fake_pitch
    global cmd_vel_lin

    #print(data.pose.pose.position.x)
    trans.translation.x = data.pose.pose.position.x
    trans.translation.y = data.pose.pose.position.y
    trans.translation.z = data.pose.pose.position.z
    trans.rotation.x = data.pose.pose.orientation.x
    trans.rotation.y = data.pose.pose.orientation.y
    trans.rotation.z = data.pose.pose.orientation.z
    trans.rotation.w = data.pose.pose.orientation.w
   
    yaw, pitch, roll = quaternion_to_euler(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w)

    # print(yaw, fake_pitch)

    rotation_z = yaw + cmd_vel.angular.z

    # print(rotation)

    cmd_vel.linear.x = cmd_vel_lin*math.cos(rotation_z)*math.sin(fake_pitch)
    cmd_vel.linear.y = cmd_vel_lin*math.sin(rotation_z)*math.sin(fake_pitch)

    cmd_vel.linear.z = cmd_vel_lin*math.cos(fake_pitch)

    cmd_vel.linear.z += -2.143
    # cmd_vel.linear.z += -2.38
    # cmd_vel.linear.z = -0.02
    
    # print("X: "+str(cmd_vel.linear.x))
    # print("Y: "+str(cmd_vel.linear.y))
    # print("z: "+str(cmd_vel.linear.z))

    point = MultiDOFJointTrajectoryPoint()
    velocity = MultiDOFJointTrajectory()

    point.transforms.append(trans)
    point.velocities.append(cmd_vel)
    velocity.points.append(point)

    pub.publish(velocity)    

def reset_fake_pitch_callback(data):
    global fake_pitch
    if (data.data == True):
        fake_pitch = pi/2

if __name__ == "__main__": 
    rospy.init_node("mission_planner_node", anonymous=False)    

    rospy.Subscriber("/hydrone_aerial_underwater/ground_truth/odometry", Odometry, position_callback)
    rospy.Subscriber("/hydrone_aerial_underwater/cmd_vel", Twist, velocity_callback)
    rospy.Subscriber("/hydrone_aerial_underwater/reset_fake_pitch", Bool, reset_fake_pitch_callback)
    # rospy.Timer(rospy.Duration(1.0), velocity_callback)

    # rospy.spin()
    global fake_pitch
    global cmd_vel
    sleep_rate = 322
    rate = rospy.Rate(sleep_rate) # ROS Rate at 5Hz

    while (not rospy.is_shutdown()):
        
        fake_pitch += cmd_vel.angular.y/sleep_rate
        if (abs(fake_pitch) > pi):
            fake_pitch = -fake_pitch

        rate.sleep()