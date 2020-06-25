#!/usr/bin/env python

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from std_msgs.msg import *
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# pathfollowing
# world = False
# if world:
#     from respawnGoal_custom_worlds import Respawn
# else:
#     from respawnGoal_3D import Respawn
# import copy
# target_not_movable = False

# Navegation
world = True
from respawnGoal_3D import Respawn
import copy
target_not_movable = True

UNDERWATER = False

class Env():
    def __init__(self, action_dim=3):
        self.goal_x = 0
        self.goal_y = 0
        self.goal_z = 0
        self.heading = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('/hydrone_aerial_underwater/cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('/hydrone_aerial_underwater/ground_truth/odometry', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_world', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.reset_fake_pitch = rospy.Publisher("/hydrone_aerial_underwater/reset_fake_pitch", Bool, queue_size=5)
        self.respawn_goal = Respawn()
        self.past_distance = 0.
        self.stopped = 0
        self.action_dim = action_dim
        #Keys CTRL + c will stop script
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        #you can stop turtlebot by publishing an empty Twist
        #message
        rospy.loginfo("Stopping Simulation")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def getGoalDistace(self):
        goal_distance = math.sqrt((self.goal_x - self.position.x)**2 + (self.goal_y - self.position.y)**2 + (self.goal_z - self.position.z)**2)
        self.past_distance = goal_distance

        return goal_distance

    def getOdometry(self, odom):
        self.past_position = copy.deepcopy(self.position)
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        #print 'yaw', yaw
        #print 'gA', goal_angle

        heading = goal_angle - yaw
        #print 'heading', heading
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 3)

    def getState(self, scan, past_action):
        global UNDERWATER
        scan_range = []
        heading = self.heading
        min_range = 0.6
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(20.0)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])


        if min_range > min(scan_range): #or self.position.z < 0.2 or self.position.z > 2.8:
            done = True

        for pa in past_action:
            scan_range.append(pa)

        current_distance = math.sqrt((self.goal_x - self.position.x)**2 + (self.goal_y - self.position.y)**2 + (self.goal_z - self.position.z)**2)
        # current_distance = math.sqrt((self.goal_x - self.position.x)**2 + (self.goal_y - self.position.y)**2)
        
        if current_distance < 0.5:
            self.get_goalbox = True

        # print(heading, current_distance)

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done):
        current_distance = state[-1]
        heading = state[-2]
        #print('cur:', current_distance, self.past_distance)

        reward = 0

        distance_rate = (self.past_distance - current_distance) 
        # if distance_rate > 0:
        #     # reward = 200.*distance_rate
        reward = 10.0*(distance_rate)

        # min_ran = min(scan.ranges)

        # if ( min_ran > 1.0 ):
        #     reward = 0.1/min_ran
        # else:
        #     reward = -1.0/min_ran

        # if (abs(self.heading) < math.pi/2):
        #     if abs(self.heading) <= 0.01:
        #         self.heading = 0.01

        #     if abs(0.1/(self.heading)) < 10.0 and (scan.ranges[0] > current_distance):
        #         reward += abs(1.0/(self.heading))/scan.ranges[0]
        #     elif abs(0.1/(self.heading)) > 10.0 and (scan.ranges[0] > current_distance):
        #         reward += 10.0/scan.ranges[0]

            # if (current_distance < 1.0):
            #     reward += 1.0/current_distance
                
        # # if (current_distance >= 1.0):
        # #     reward += -0.1*current_distance

        # print(abs(1.0/(self.heading)), scan.ranges[0], current_distance)
        # print(reward)
        # # # if distance_rate == 0:
        # # #     reward = 0.

        # if distance_rate <= 0:
        #     # reward = -8.
        #     reward = -1.0/current_distance.

        # print(reward)

        #angle_reward = math.pi - abs(heading)
        #print('d', 500*distance_rate)
        #reward = 500.*distance_rate #+ 3.*angle_reward
        self.past_distance = current_distance

        # a, b, c, d = float('{0:.3f}'.format(self.position.x)), float('{0:.3f}'.format(self.past_position.x)), float('{0:.3f}'.format(self.position.y)), float('{0:.3f}'.format(self.past_position.y))
        # if a == b and c == d:
        #     # rospy.loginfo('\n<<<<<Stopped>>>>>\n')
        #     # print('\n' + str(a) + ' ' + str(b) + ' ' + str(c) + ' ' + str(d) + '\n')
        #     self.stopped += 1
        #     print(self.stopped)
        #     if self.stopped == 20:
        #         rospy.loginfo('Robot is in the same 10 times in a row')
        #         self.stopped = 0
        #         done = True
        # data = None
        # while data is None:
        #     try:
        #         data = rospy.wait_for_message('/hydrone_aerial_underwater/scan', LaserScan, timeout=5)
        #     except:
        #         pass
        # print(data.ranges)

        # if (min(data.ranges) < 0.6):            
        #     done = True
        # else:
        #     # rospy.loginfo('\n>>>>> not stopped>>>>>\n')
        #     self.stopped = 0

        if done:
            rospy.loginfo("Collision!!")
            # reward = -550.
            reward = -10.
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!! "+str(abs(self.goal_z - self.position.z)))
            # reward = 500.
            reward = 100#/(abs(self.goal_z - self.position.z)+0.01)
            self.pub_cmd_vel.publish(Twist())
            if world and target_not_movable:
                self.reset()
            self.goal_x, self.goal_y, self.goal_z = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward, done

    def step(self, action, past_action):
        global UNDERWATER
        linear_vel_x = action[0]        
        angular_vel_y = action[1]
        angular_vel_z = action[2]
        # angular_vel_z = action[2]

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel_x
        vel_cmd.angular.y = angular_vel_y     
        vel_cmd.angular.z = angular_vel_z

        self.pub_cmd_vel.publish(vel_cmd)

        data = None

        if (UNDERWATER):
            laserscan = LaserScan()
            for i in range(0, 8):
                data = None
                # print('/hydrone_aerial_underwater/sensor/ir_front_'+str(i*45))
                while data is None:                    
                    try:
                        data = rospy.wait_for_message('/hydrone_aerial_underwater/sensor/ir_front_'+str(i*45), Range, timeout=5)
                    except:
                        pass

                laserscan.ranges.append(data.range)
                # print(data.range)
                # print("----------------")

            data = laserscan
            # print(data)                
        else:
            while data is None:
                try:
                    data = rospy.wait_for_message('/hydrone_aerial_underwater/scan', LaserScan, timeout=5)
                except:
                    pass

        state, done = self.getState(data, past_action)
        reward, done = self.setReward(state, done)

        return np.asarray(state), reward, done

    def reset(self):
        #print('aqui2_____________---')
        global UNDERWATER
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        if (UNDERWATER):
            laserscan = LaserScan()
            for i in range(0, 8):
                while data is None:
                    # print('/hydrone_aerial_underwater/sensor/ir_front_'+str(i*45))
                    try:
                        data = rospy.wait_for_message('/hydrone_aerial_underwater/sensor/ir_front_'+str(i*45), Range, timeout=5)
                    except:
                        pass

                laserscan.ranges.append(data.range)

            data = laserscan
            # print(data)

        else:
            while data is None:
                try:
                    data = rospy.wait_for_message('/hydrone_aerial_underwater/scan', LaserScan, timeout=5)
                except:
                    pass

        self.reset_fake_pitch.publish(True)

        if self.initGoal:
            self.goal_x, self.goal_y, self.goal_z = self.respawn_goal.getPosition()
            self.initGoal = False
        else:
            self.goal_x, self.goal_y, self.goal_z = self.respawn_goal.getPosition(True, delete=True)

        self.goal_distance = self.getGoalDistace()
        # state, _ = self.getState(data, [0.,0., 0.0])
        state, _ = self.getState(data, [0]*self.action_dim)

        return np.asarray(state)