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
from std_srvs.srv import Empty
from datetime import datetime
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pub = rospy.Publisher('/hydrone_aerial_underwater/command/pose', PoseStamped, queue_size=10)

pub_cmd_vel = rospy.Publisher('/hydrone_aerial_underwater/cmd_vel', Twist, queue_size=5)        
pub_end = rospy.Publisher("/hydrone_aerial_underwater/end_testing", Bool, queue_size=5)
eps_to_test = 100
counter_eps = 0
last_time = datetime.now() 
pub_reward = rospy.Publisher("/hydrone_aerial_underwater/rewarded", Bool, queue_size=5)

posx = [3.6, 0.0]
posy = [3.6, 3.0]
posz = [2.5, 2.5]

_data = Odometry()
scan = LaserScan()

def position_callback(data):
    global _data
    _data = data

def laser_callback(data):
    global scan
    scan = data

def publish_velocity(linear_vel_x, angular_vel_z):
    vel_cmd = Twist()
    vel_cmd.linear.x = linear_vel_x
    # vel_cmd.linear.y = linear_vel_y
    vel_cmd.angular.z = angular_vel_z

    pub_cmd_vel.publish(vel_cmd)

def get_yaw():
    global _data
    orientation = _data.pose.pose.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(orientation_list)

    return yaw


def rotate(target_x, target_y):
    goal_angle = math.atan2(target_y - _data.pose.pose.position.y, target_x - _data.pose.pose.position.x)
    yaw = get_yaw()
    
    while (abs(yaw-goal_angle) > 0.1):
        goal_angle = math.atan2(target_y - _data.pose.pose.position.y, target_x - _data.pose.pose.position.x)
        yaw = get_yaw()  

        print(abs(yaw-goal_angle))   

        if (yaw-goal_angle < 0):
            publish_velocity(0.0, 0.25)
        else:
            publish_velocity(0.0, -0.25)

def go_forward(target_x, target_y):
    global _data
    global scan
    global pub_cmd_vel
    global posx
    global posy
    global posz

    while True:
        distance = math.sqrt((target_x - _data.pose.pose.position.x)**2 + (target_y - _data.pose.pose.position.y)**2)# + (posz[0] - _data.pose.pose.position.z)**2)
        # print(distance)    

        if (distance < 0.5):
            return True
        elif (min(scan.ranges) < 0.75):
            publish_velocity(0.0, 0.0)
            return False 
        else:            
            publish_velocity(0.25, 0.0)            

        rotate(target_x, target_y)

def rotate_to_contour():
    i = scan.ranges.index(min(scan.ranges))
    vel_cmd = Twist()

    while True:
        publish_velocity(0.0, -0.25)
        print(i)

def reset():
    global counter_eps
    global pub_reward
    global last_time
    global pub_cmd_vel

    rospy.wait_for_service('gazebo/reset_simulation')

    try:
        reset_proxy = rospy.ServiceProxy('gazebo/reset_world', Empty)
        reset_proxy()
    except (rospy.ServiceException) as e:
        print("gazebo/reset_simulation service call failed")

    timer = Twist()
    timer.linear.y = (datetime.now() - last_time).total_seconds()
    pub_cmd_vel.publish(timer)
    last_time = datetime.now()
    
    pub_reward.publish(True)

    time.sleep(2)

if __name__ == "__main__":   
    global posx
    global posy
    global posz
    global pub
    global pub_cmd_vel
    global last_time
    global eps_to_test
    global pub_end
    global pub_reward
    global _data
    global scan

    rospy.init_node("test_bug", anonymous=False)    

    rospy.Subscriber("/hydrone_aerial_underwater/ground_truth/odometry", Odometry, position_callback)

    rospy.Subscriber("/hydrone_aerial_underwater/scan", LaserScan, laser_callback)

    # rospy.Subscriber("/hydrone_aerial_underwater/next_position_pid", Int64, state_callback)

    # for each episode
        # get distance towards the goal
        # rotate until lasers 540 towards the goal
        # go forward
        # if distance to the goal less than 0.5 finishes episode
        # if found obstacle, rotate to the left until lasers 540 distance less then distance to the goal

    for i in range(0, eps_to_test): 
        while len(scan.ranges) == 0:
            rospy.loginfo("Waiting for laser") 

        for j in range (0, len(posx)): 

            rotate(posx[j], posy[j])

            if (not go_forward(posx[j], posy[j])):
                rotate_to_contour()
                # rotate until laser 900 the least
                # contour
                    # if laser 540 distance less then distance to the goal
                        # break
                # if go_forward
                    # break
        reset()


    # while not rospy.is_shutdown():      

    #     distance = math.sqrt((posx[0] - _data.pose.pose.position.x)**2 + (posy[0] - _data.pose.pose.position.y)**2)# + (posz[0] - _data.pose.pose.position.z)**2)
        
    #     pose = PoseStamped()
    #     pose.pose.position.x = posx[0]
    #     pose.pose.position.y = posy[0]
    #     # pose.pose.position.z = posz[0]

    #     while len(scan.ranges) == 0:
    #         rospy.loginfo("Waiting for laser")

    #     if (distance < 0.5 or min(scan.ranges) < 0.6):
    #         # i += 1
    #         # print(i)

    #         rospy.wait_for_service('gazebo/reset_simulation')
    #         try:
    #             reset_proxy = rospy.ServiceProxy('gazebo/reset_world', Empty)
    #             reset_proxy()
    #         except (rospy.ServiceException) as e:
    #             print("gazebo/reset_simulation service call failed")

    #         timer = Twist()
    #         timer.linear.y = (datetime.now() - last_time).total_seconds()
    #         pub_cmd_vel.publish(timer)
    #         last_time = datetime.now()

    #         if (distance < 0.25):
    #             pub_reward.publish(True)
            
    #         counter_eps += 1

    #         if(counter_eps == eps_to_test):
    #             pub_end.publish(False)
    #             rospy.signal_shutdown("end_test")

    #         time.sleep(2)

    #     pub.publish(pose)

# roslaunch hydrone_aerial_underwater_ddpg lee_controller.launch root_dir:=/home/ricardo/ file_dir:=lee_stage_1 testing:=true world:=stage_1

    