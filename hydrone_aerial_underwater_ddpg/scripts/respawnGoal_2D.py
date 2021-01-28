#!/usr/bin/env python

import rospy
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import math

class Respawn():
    def __init__(self):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('hydrone_aerial_underwater_ddpg/scripts',
                                                'hydrone_aerial_underwater_ddpg/models/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = 1
        self.goal_position = Pose()
        self.init_goal_x = rospy.get_param('~x_start')
        self.init_goal_y = rospy.get_param('~y_start')        
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'
        self.obstacle_1 = 2.0, 2.0
        self.obstacle_2 = 2.0, -2.0
        self.obstacle_3 = -2.0, 2.0
        self.obstacle_4 = -2.0, -2.0
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0
        self.evaluating = rospy.get_param('~test_param')
        self.eval_path = rospy.get_param('~eval_path')
        self.eval_scenario_2 = rospy.get_param('~scenario_2')

        if (self.eval_scenario_2):
            self.goal_x_list = [3.6, 0.0, -3.6, -3.6, 0.0]
            self.goal_y_list = [2.6, 3.5, 3.0, 1.0, 0.0]
        else:
            self.goal_x_list = [2.0, 0.0, -2.0, -2.0, 0.0, 2.0, 0.0]
            self.goal_y_list = [2.0, 3.0, 2.0, -2.0, -3.0, -2.0, 0.0]

        self.counter = 0

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                self.goal_position.position.z = 2.5
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x,
                              self.goal_position.position.y)
                break
            else:
                pass
        
        self.counter += 1

    def deleteModel(self):
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break
            else:
                pass

        # self.index = 0

    def getPosition(self, position_check=False, delete=False):
    
        if delete:
            self.deleteModel()

        if self.stage != 4 and self.evaluating == False:
            while position_check:
                goal_x = random.randrange(-40, 40) / 10.0
                goal_y = random.randrange(-40, 40) / 10.0
                if abs(goal_x - self.obstacle_1[0]) <= 1.0 and abs(goal_y - self.obstacle_1[1]) <= 1.0:
                    position_check = True
                elif abs(goal_x - self.obstacle_2[0]) <= 1.0 and abs(goal_y - self.obstacle_2[1]) <= 1.0:
                    position_check = True
                elif abs(goal_x - self.obstacle_3[0]) <= 1.0 and abs(goal_y - self.obstacle_3[1]) <= 1.0:
                    position_check = True
                elif abs(goal_x - self.obstacle_4[0]) <= 1.0 and abs(goal_y - self.obstacle_4[1]) <= 1.0:
                    position_check = True
                elif abs(goal_x - 0.0) <= 0.6 and abs(goal_y - 0.0) <= 0.6:
                    position_check = True
                else:
                    position_check = False

                if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                    position_check = True

                self.goal_position.position.x = goal_x
                self.goal_position.position.y = goal_y

        if (self.evaluating and self.eval_path):
            self.goal_position.position.x = self.goal_x_list[self.counter%len(self.goal_x_list)]
            self.goal_position.position.y = self.goal_y_list[self.counter%len(self.goal_x_list)]
            rospy.loginfo("Counter: %s", str(self.counter%len(self.goal_x_list)))
            
        # goal_x_list = [2.0, 0.0, -2.0, -2.0, 0.0, 2.0, 0.0]
        # goal_y_list = [2.0, 3.0, 2.0, -2.0, -3.0, -2.0, 0.0]

        # # goal_x_list = [3.6, 0.0, -3.6, -3.6, 0.0]
        # # goal_y_list = [2.6, 3.5, 3.0, 1.0, 0.0]
        
        # self.goal_position.position.x = goal_x_list[self.index]
        # self.goal_position.position.y = goal_y_list[self.index]

        # self.index += 1

        # if self.index < 70:

        #     t = float(self.index) / 25 * math.pi
        #     dx = (0.5*t) * math.cos(t)
        #     dy = (0.5*t) * math.sin(t)

        #     print(dx,dy)

        #     self.goal_position.position.x = dx
        #     self.goal_position.position.y = dy
        
        #     self.index += 1
        #     print(self.index)

        time.sleep(0.5)
        self.respawnModel()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y
       
        return self.goal_position.position.x, self.goal_position.position.y
