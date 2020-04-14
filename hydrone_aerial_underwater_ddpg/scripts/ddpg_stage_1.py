#!/usr/bin/env python
# Authors: Junior Costa de Jesus #

import rospy
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import Float32
from environment_stage_1 import Env
import torch
import torch.nn.functional as F
import gc
import torch.nn as nn
import math
from collections import deque
import copy


#---Directory Path---#
dirPath = os.path.dirname(os.path.realpath(__file__))
#---Functions to make network updates---#
 
def soft_update(target, source, tau):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(target_param.data*(1.0 - tau)+ param.data*tau)

def hard_update(target,source):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(param.data)

#---Ornstein-Uhlenbeck Noise for action---#

class ActionNoise:
    # Based on http://math.stackexchange.com/questions/1287634/implementing-ornstein-uhlenbeck-in-matlab
    def __init__(self, action_dim, mu=0, theta=0.15, sigma=0.2):
        self.action_dim = action_dim
        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.X = np.ones(self.action_dim)*self.mu
        
    def reset(self):
        self.X = np.ones(self.action_dim)*self.mu
    
    def sample(self):
        dx = self.theta*(self.mu - self.X)
        dx = dx + self.sigma*np.random.randn(len(self.X))
        self.X = self.X + dx
        return self.X

#---Critic--#

EPS = 0.003
def fanin_init(size, fanin=None):
    fanin = fanin or size[0]
    v = 1./np.sqrt(fanin)
    return torch.Tensor(size).uniform_(-v,v)

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        
        self.state_dim = state_dim = state_dim
        self.action_dim = action_dim
        
        self.fc1 = nn.Linear(state_dim, 256)
        self.fc1.weight.data = fanin_init(self.fc1.weight.data.size())
        
        self.fa1 = nn.Linear(action_dim, 256)
        self.fa1.weight.data = fanin_init(self.fa1.weight.data.size())
        
        self.fca1 = nn.Linear(512, 512)
        self.fca1.weight.data = fanin_init(self.fca1.weight.data.size())
        
        self.fca2 = nn.Linear(512, 1)
        self.fca2.weight.data.uniform_(-EPS, EPS)
        
    def forward(self, state, action):
        xs = mish(self.fc1(state))
        xa = mish(self.fa1(action))
        x = torch.cat((xs,xa), dim=1)
        x = mish(self.fca1(x))
        vs = self.fca2(x)
        return vs

#---Actor---#

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, action_limit_v,  action_limit_w):
        super(Actor, self).__init__()
        self.state_dim = state_dim = state_dim
        self.action_dim = action_dim
        self.action_limit_v = action_limit_v
        self.action_limit_w = action_limit_w
        
        self.fa1 = nn.Linear(state_dim, 512)
        self.fa1.weight.data = fanin_init(self.fa1.weight.data.size())
        
        self.fa2 = nn.Linear(512, 512)
        self.fa2.weight.data = fanin_init(self.fa2.weight.data.size())
        
        self.fa3 = nn.Linear(512, action_dim)
        self.fa3.weight.data.uniform_(-EPS,EPS)
        
    def forward(self, state):
        # print(state)
        x = mish(self.fa1(state))
        x = mish(self.fa2(x))
        action = self.fa3(x)
        if state.shape == torch.Size([14]):
            action[0] = torch.sigmoid(action[0])*self.action_limit_v
            action[1] = torch.tanh(action[1])*self.action_limit_w
        else:
            action[:,0] = torch.sigmoid(action[:,0])*self.action_limit_v
            action[:,1] = torch.tanh(action[:,1])*self.action_limit_w
        return action

#---Memory Buffer---#

class MemoryBuffer:
    def __init__(self, size):
        self.buffer = deque(maxlen=size)
        self.maxSize = size
        self.len = 0
        
    def sample(self, count):
        batch = []
        count = min(count, self.len)
        batch = random.sample(self.buffer, count)
        
        s_array = np.float32([array[0] for array in batch])
        a_array = np.float32([array[1] for array in batch])
        r_array = np.float32([array[2] for array in batch])
        new_s_array = np.float32([array[3] for array in batch])
        
        return s_array, a_array, r_array, new_s_array
    
    def len(self):
        return self.len
    
    def add(self, s, a, r, new_s):
        transition = (s, a, r, new_s)
        self.len += 1 
        if self.len > self.maxSize:
            self.len = self.maxSize
        self.buffer.append(transition)

#---Where the train is made---#

BATCH_SIZE = 128
LEARNING_RATE = 0.001
GAMMA = 0.99
TAU = 0.001

class Trainer:
    
    def __init__(self, state_dim, action_dim, action_limit_v, action_limit_w, ram):
        
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.action_limit_v = action_limit_v
        self.action_limit_w = action_limit_w
        #print('w',self.action_limit_w)
        self.ram = ram
        #self.iter = 0 
        self.noise = ActionNoise(self.action_dim)
        
        self.actor = Actor(self.state_dim, self.action_dim, self.action_limit_v, self.action_limit_w)
        self.target_actor = Actor(self.state_dim, self.action_dim, self.action_limit_v, self.action_limit_w)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), LEARNING_RATE)
        
        self.critic = Critic(self.state_dim, self.action_dim)
        self.target_critic = Critic(self.state_dim, self.action_dim)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), LEARNING_RATE)
        self.pub_qvalue = rospy.Publisher('qvalue', Float32, queue_size=5)
        self.qvalue = Float32()
        
        hard_update(self.target_actor, self.actor)
        hard_update(self.target_critic, self.critic)
        
    def get_exploitation_action(self,state):
        state = torch.from_numpy(state)
        action = self.target_actor.forward(state).detach()
        #print('actionploi', action)
        return action.data.numpy()
        
    def get_exploration_action(self, state):
        state = torch.from_numpy(state)
        action = self.actor.forward(state).detach()
        #noise = self.noise.sample()
        #print('noisea', noise)
        #noise[0] = noise[0]*self.action_limit_v
        #noise[1] = noise[1]*self.action_limit_w
        #print('noise', noise)
        new_action = action.data.numpy() #+ noise
        #print('action_no', new_action)
        return new_action
    
    def optimizer(self):
        s_sample, a_sample, r_sample, new_s_sample = ram.sample(BATCH_SIZE)
        
        s_sample = torch.from_numpy(s_sample)
        a_sample = torch.from_numpy(a_sample)
        r_sample = torch.from_numpy(r_sample)
        new_s_sample = torch.from_numpy(new_s_sample)
        
        #-------------- optimize critic
        
        a_target = self.target_actor.forward(new_s_sample).detach()
        next_value = torch.squeeze(self.target_critic.forward(new_s_sample, a_target).detach())
        # y_exp = r _ gamma*Q'(s', P'(s'))
        y_expected = r_sample + GAMMA*next_value
        # y_pred = Q(s,a)
        y_predicted = torch.squeeze(self.critic.forward(s_sample, a_sample))
        #-------Publisher of Vs------
        self.qvalue = y_predicted.detach()
        self.pub_qvalue.publish(torch.max(self.qvalue))
        #print(self.qvalue, torch.max(self.qvalue))
        #----------------------------
        loss_critic = F.smooth_l1_loss(y_predicted, y_expected)

        
        self.critic_optimizer.zero_grad()
        loss_critic.backward()
        self.critic_optimizer.step()
        
        #------------ optimize actor
        pred_a_sample = self.actor.forward(s_sample)
        loss_actor = -1*torch.sum(self.critic.forward(s_sample, pred_a_sample))
        
        self.actor_optimizer.zero_grad()
        loss_actor.backward()
        self.actor_optimizer.step()
        
        soft_update(self.target_actor, self.actor, TAU)
        soft_update(self.target_critic, self.critic, TAU)
    
    def save_models(self, episode_count):
        torch.save(self.target_actor.state_dict(), dirPath +'/Models/' + world + '/' + str(episode_count)+ '_actor.pt')
        torch.save(self.target_critic.state_dict(), dirPath + '/Models/' + world + '/'+str(episode_count)+ '_critic.pt')
        print('****Models saved***')
        
    def load_models(self, episode):
        self.actor.load_state_dict(torch.load(dirPath + '/Models/' + world + '/'+str(episode)+ '_actor.pt'))
        self.critic.load_state_dict(torch.load(dirPath + '/Models/' + world + '/'+str(episode)+ '_critic.pt'))
        hard_update(self.target_actor, self.actor)
        hard_update(self.target_critic, self.critic)
        print('***Models load***')

#---Mish Activation Function---#
def mish(x):
    '''
        Mish: A Self Regularized Non-Monotonic Neural Activation Function
        https://arxiv.org/abs/1908.08681v1
        implemented for PyTorch / FastAI by lessw2020
        https://github.com/lessw2020/mish
        param:
            x: output of a layer of a neural network
        return: mish activation function
    '''
    return x*(torch.tanh(F.softplus(x)))

#---Run agent---#

is_training = True

exploration_decay_rate = 0.001

MAX_EPISODES = 10001
MAX_STEPS = 100
MAX_BUFFER = 50000
# rewards_all_episodes = []

STATE_DIMENSION = 14
ACTION_DIMENSION = 2
ACTION_VX_MAX = 0.5 # m/s
ACTION_W_MAX = 1.0 # rad/s
world = 'stage_1'

if is_training:
    var_vx = ACTION_VX_MAX
    var_w = ACTION_W_MAX
else:
    var_vx = ACTION_VX_MAX*0.10
    var_w = ACTION_W_MAX*0.10

print('State Dimensions: ' + str(STATE_DIMENSION))
print('Action Dimensions: ' + str(ACTION_DIMENSION))
print('Action Max: ' + str(ACTION_VX_MAX) + ' m/s and ' + str(ACTION_W_MAX) + ' rad/s')
ram = MemoryBuffer(MAX_BUFFER)
trainer = Trainer(STATE_DIMENSION, ACTION_DIMENSION, ACTION_VX_MAX, ACTION_W_MAX, ram)
#trainer.load_models(1560)

if __name__ == '__main__':
    rospy.init_node('ddpg_stage_1')
    pub_result = rospy.Publisher('result', Float32, queue_size=5)
    result = Float32()
    env = Env()
    before_training = 1

    print("test")

    past_action = np.array([0.,0.])

    for ep in range(MAX_EPISODES):
        done = False
        state = env.reset()
        if is_training and ep%2 == 0 and ram.len >= before_training*MAX_STEPS:
            print('Episode: ' + str(ep) + ' training')
        else:
            if ram.len >= before_training*MAX_STEPS:
                print('Episode: ' + str(ep) + ' evaluating')
            else:
                print('Episode: ' + str(ep) + ' adding to memory')

        rewards_current_episode = 0.

        for step in range(MAX_STEPS):
            state = np.float32(state)

            if is_training and ep% 2 == 0 and ram.len >= before_training*MAX_STEPS:
                action = trainer.get_exploration_action(state)
                action[0] = np.clip(np.random.normal(action[0], var_vx), 0.0, ACTION_VX_MAX)
                action[1] = np.clip(np.random.normal(action[1], var_w), -ACTION_W_MAX, ACTION_W_MAX)
            else:
                action = trainer.get_exploitation_action(state)

            if not is_training:
                action = trainer.get_exploitation_action(state)
            next_state, reward, done = env.step(action, past_action)
            # print('action', action,'r',reward)
            past_action = action

            rewards_current_episode += reward
            next_state = np.float32(next_state)
            if ep%2 == 0 or not ram.len >= before_training*MAX_STEPS:
                if reward == 100.:
                    print('***\n-------- Maximum Reward ----------\n****')
                    for _ in range(3):
                        ram.add(state, action, reward, next_state)
                else:
                    ram.add(state, action, reward, next_state)
            state = copy.deepcopy(next_state)
            

            if ram.len >= before_training*MAX_STEPS and is_training and ep% 2 == 0:
                var_vx = max([var_vx*0.99995, 0.1*ACTION_VX_MAX])
                var_w = max([var_w*0.99995, 0.1*ACTION_W_MAX])
                trainer.optimizer()

            if done or step == MAX_STEPS-1:
                print('reward per ep: ' + str(rewards_current_episode))
                print('reward average per ep: ' + str(round(rewards_current_episode/step, 2)) + ' and break step: ' + str(step))
                print('explore_vx: ' + str(var_vx) + ' and explore_w: ' + str(var_w))
                # rewards_all_episodes.append(rewards_current_episode)
                if (ep)% 2 == 0:
                    pass
                else:
                    if ram.len >= before_training*MAX_STEPS:
                        result = round(rewards_current_episode/step, 2)
                        pub_result.publish(result)
                break
        if ep%20 == 0:
            trainer.save_models(ep)

print('Completed Training')
