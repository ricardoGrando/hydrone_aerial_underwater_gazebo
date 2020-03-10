#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geographic_msgs.msg import *
from trajectory_msgs.msg import *
import math
from uuv_gazebo_ros_plugins_msgs.msg import *

set_aerial_point_pub = rospy.Publisher("/hydrone_aerial_underwater/command/pose", PoseStamped, queue_size=10)
enable_aerial_pub = rospy.Publisher("/hydrone_aerial_underwater/command/pose/enable", Bool, queue_size=10)
enable_underwater_pub = rospy.Publisher("/hydrone_aerial_underwater/command/thrust/enable", FloatStamped, queue_size=10)
thruster_pub = []

for i in range(0, 4):
    thruster_pub.append(rospy.Publisher("/hydrone_aerial_underwater/thrusters/"+str(i)+"/input", FloatStamped, queue_size=10))

def set_target_position(x,y,z):   
    # if abs(x) < 0.05: x = 0.05
    # if abs(y) < 0.05: y = 0.05
    # if abs(z) < 0.05: z = 0.05
    # if z <= 0.2: offset_altitude = 0.0
    # else: offset_altitude = OFFSET_Z

    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    # while True:                      
    #     value = rospy.wait_for_message("/hydrone_aerial_underwater/odometry_sensor1/pose", Pose)
    #     # Closed Loop            
    #     if ( (value.position.x/x > 1.0-ARR_THRES) and (value.position.x/x < 1.0+ARR_THRES) and \
    #             (value.position.y/y > 1.0-ARR_THRES) and (value.position.y/y < 1.0+ARR_THRES) and \
    #             ((value.position.z+offset_altitude)/z > 1.0-ARR_THRES) and ((value.position.z+offset_altitude)/z < 1.0+ARR_THRES)):
    #         # print("Arrived to target point")               
            
    #         break                
    #     else:
    set_aerial_point_pub.publish(pose)


if __name__ == "__main__": 
    rospy.init_node("hydrone_aerial_underwater_mission_planner_node", anonymous=False)    

    time.sleep(2.0)
    msg = FloatStamped()
    msg.data = 0.0
    print("Disabling undewater thrusters mode...")
    enable_underwater_pub.publish(msg)
    time.sleep(1.0)
    print("Enabling aerial thrusters mode...")
    enable_aerial_pub.publish(True)
    time.sleep(1.0)
    print("Took off...")
    print("Going to first position....")
    set_target_position(-12.0, 2.0, 5.0)
    time.sleep(5.0)
    print("Going to second position....")
    set_target_position(-12.0, 10.0, 5.0)
    time.sleep(5.0)
    set_target_position(-12.0, 10.0, 2.0)
    time.sleep(5.0)
    set_target_position(-12.0, 10.0, -0.6)
    time.sleep(5.0)    
    print("Disabling aerial thrusters mode...")
    enable_aerial_pub.publish(False)
    print("Landing on the water...")
    time.sleep(10.0)
    
    msg.data = -1.0
    print("Activating undewater thrusters mode...")
    enable_underwater_pub.publish(msg)    

    print("Diving....")
    msg.data = -100.0
    for i in range(0, len(thruster_pub)):
        thruster_pub[i].publish(msg)
    time.sleep(5.0)
    
    print("Felling the water....")
    msg.data = 0.0
    for i in range(0, len(thruster_pub)):
        thruster_pub[i].publish(msg)
    time.sleep(20.0)

    msg.data = 0.0
    print("Disabling undewater thrusters mode...")
    enable_underwater_pub.publish(msg)
    time.sleep(2.0)
    print("Enabling aerial thrusters mode...")
    enable_aerial_pub.publish(True)
    print("Emerging....")
    print("Going to second position....")    
    set_target_position(-12.0, 10.0, 2.0)
    time.sleep(10.0)
    set_target_position(-12.0, 10.0, 5.0)
    time.sleep(5.0)
    print("Going to second position....")
    set_target_position(-12.0, 10.0, 5.0)
    time.sleep(5.0)
    print("Going to first position....")
    set_target_position(-12.0, 2.0, 5.0)
    time.sleep(5.0)
    print("Approaching home....")
    set_target_position(-12.0, 2.0, 3.0)
    time.sleep(5.0)
    print("Disabling aerial plugin...")
    enable_aerial_pub.publish(False)
    print("Landing...")
    time.sleep(1.0)
    
       



