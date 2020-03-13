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

set_point_pub = rospy.Publisher("/hydrone_aerial_underwater/command/pose", PoseStamped, queue_size=10)
enable_controller_pub = rospy.Publisher("/hydrone_aerial_underwater/command/pose/enable", Bool, queue_size=10)
enable_uuv_sim_pub = rospy.Publisher("/hydrone_aerial_underwater/command/uuv_sim/enable", Bool, queue_size=10)

# thruster_pub = []
# for i in range(0, 4):
#     thruster_pub.append(rospy.Publisher("/hydrone_aerial_underwater/thrusters/"+str(i)+"/input", FloatStamped, queue_size=10))

ARR_THRES = 0.05

def set_target_position(x,y,z):   
    if abs(x) < 0.05: x = 0.05
    if abs(y) < 0.05: y = 0.05
    if abs(z) < 0.05: z = 0.05
    # if z <= 0.2: offset_altitude = 0.0
    # else: offset_altitude = OFFSET_Z

    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    while True:                      
        value = rospy.wait_for_message("/hydrone_aerial_underwater/odometry_sensor1/pose", Pose)
        # Closed Loop            
        if ( (value.position.x/x > 1.0-ARR_THRES) and (value.position.x/x < 1.0+ARR_THRES) and \
                (value.position.y/y > 1.0-ARR_THRES) and (value.position.y/y < 1.0+ARR_THRES) and \
                ((value.position.z)/z > 1.0-ARR_THRES) and ((value.position.z)/z < 1.0+ARR_THRES)
                ):
            # print("Arrived to target point")               
            
            break                
        else:
            set_point_pub.publish(pose)

        if ((value.position.z)/z < 1.0-ARR_THRES):
            pose.pose.position.z += 0.01
            print(pose.pose.position.z)
        if ((value.position.z)/z > 1.0+ARR_THRES):
            pose.pose.position.z -= 0.01
            print(pose.pose.position.z)

if __name__ == "__main__": 
    rospy.init_node("hydrone_aerial_underwater_mission_planner_node", anonymous=False)    

    time.sleep(10.0)    
    print("Enabling thrusters...")
    enable_controller_pub.publish(True)
    time.sleep(1.0)
    print("Took off...")
    print("Going to first position....")
    set_target_position(-12.0, 2.0, 5.0)
    time.sleep(5.0)
    print("Going to second position....")
    set_target_position(-12.0, 10.0, 5.0)
    time.sleep(5.0)
    print("Hovering water....")
    set_target_position(-12.0, 10.0, 0.0)
    time.sleep(5.0)
    # set_target_position(-12.0, 10.0, -1.0)
    # time.sleep(5.0)      

    print("Disabling thrusters...")
    enable_controller_pub.publish(False)
    print("Landing on the water...")
    time.sleep(0.5)   
    
    print("Enabling thrusters...")
    enable_controller_pub.publish(True)
    print("Emerging....")
    set_target_position(-12.0, 10.0, -5.0)
    time.sleep(10.0)    
    set_target_position(-10.0, 10.0, -5.6)
    time.sleep(10.0)   
    set_target_position(-12.0, 10.0, -5.6)
    time.sleep(10.0)  
    set_target_position(-12.0, 10.0, -3.6)
    time.sleep(10.0)  
    # print("Disabling thrusters...")
    # enable_controller_pub.publish(False)

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
    print("Landing...")
    enable_controller_pub.publish(False)    
    time.sleep(1.0)
    
       



