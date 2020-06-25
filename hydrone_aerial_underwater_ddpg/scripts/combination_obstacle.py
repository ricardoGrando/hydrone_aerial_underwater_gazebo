#!/usr/bin/env python

import rospy
import time
from gazebo_msgs.msg import ModelState, ModelStates
import random

class Combination():
    def __init__(self):
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.moving()

    def moving(self):
        state_1 = 0
        state_2 = 0
        state_3 = 0
        state_4 = 0

        max_pos = 3.0
        min_pos = 1.0

        while not rospy.is_shutdown():
            model = rospy.wait_for_message('gazebo/model_states', ModelStates)
            for i in range(len(model.name)):
                if model.name[i] == 'obstacle_1':
                    obstacle_1 = ModelState()
                    obstacle_1.model_name = model.name[i]
                    obstacle_1.pose = model.pose[i]
                    if (obstacle_1.pose.position.y < max_pos and (state_1 == 0)) :
                        obstacle_1.pose.position.y += 0.01
                        if (obstacle_1.pose.position.y >= max_pos):
                            state_1 = 1
                    elif (obstacle_1.pose.position.y > min_pos and (state_1 == 1)) :
                        obstacle_1.pose.position.y -= 0.01
                        if (obstacle_1.pose.position.y <= min_pos):
                            state_1 = 0
                    
                    self.pub_model.publish(obstacle_1)

                if model.name[i] == 'obstacle_2':
                    obstacle_1 = ModelState()
                    obstacle_1.model_name = model.name[i]
                    obstacle_1.pose = model.pose[i]
                    if (obstacle_1.pose.position.y > -max_pos and (state_2 == 0)) :
                        obstacle_1.pose.position.y += -0.01
                        if (obstacle_1.pose.position.y <= -max_pos):
                            state_2 = 1
                    elif (obstacle_1.pose.position.y < -min_pos and (state_2 == 1)) :
                        obstacle_1.pose.position.y += 0.01
                        if (obstacle_1.pose.position.y >= -min_pos):
                            state_2 = 0
                    
                    self.pub_model.publish(obstacle_1)

                if model.name[i] == 'obstacle_3':
                    obstacle_1 = ModelState()
                    obstacle_1.model_name = model.name[i]
                    obstacle_1.pose = model.pose[i]
                    if (obstacle_1.pose.position.x < max_pos and (state_3 == 0)) :
                        obstacle_1.pose.position.x += 0.01
                        if (obstacle_1.pose.position.x >= max_pos):
                            state_3 = 1
                    elif (obstacle_1.pose.position.x > min_pos and (state_3 == 1)) :
                        obstacle_1.pose.position.x -= 0.01
                        if (obstacle_1.pose.position.x <= min_pos):
                            state_3 = 0
                    
                    self.pub_model.publish(obstacle_1)
                
                if model.name[i] == 'obstacle_4':
                    obstacle_1 = ModelState()
                    obstacle_1.model_name = model.name[i]
                    obstacle_1.pose = model.pose[i]
                    if (obstacle_1.pose.position.x > -max_pos and (state_4 == 0)) :
                        obstacle_1.pose.position.x += -0.01
                        if (obstacle_1.pose.position.x <= -max_pos):
                            state_4 = 1
                    elif (obstacle_1.pose.position.x < -min_pos and (state_4 == 1)) :
                        obstacle_1.pose.position.x += 0.01
                        if (obstacle_1.pose.position.x >= -min_pos):
                            state_4 = 0
                    
                    self.pub_model.publish(obstacle_1)
                
            time.sleep(0.05)

def main():
    rospy.init_node('combination_obstacle_1')
    try:
        combination = Combination()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
