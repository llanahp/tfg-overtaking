# Import GYM 
from turtle import end_fill, st
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete

# Import helpers
import os, sys
import numpy as np

#Import SUMO
from tkinter.messagebox import NO
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

class CustomEnv(Env):
    def __init__(self, render=False):
        self.N_actions = 3
        self.action_space = Discrete(self.N_actions)
        self.observation_space = Dict({'ego':Box(0,1,shape=(1,)), 'adversaries':Box(0,1,shape=(12,)),})
        self.state = self.observation_space.sample()

        self.state = self.reset()

    def step(self, action):

        self.state = self.observation_space.sample()
        done = False
        reward = self._reward()
        info = {}

        if done:
            reward = 0

        return self.state, reward, done, info


    def reset(self):

        self.state = self.observation_space.sample()

        return self.state

    
    def _reward(self):
      
        reward = 1

        return reward
