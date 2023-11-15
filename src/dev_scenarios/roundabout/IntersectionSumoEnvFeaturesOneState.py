# Import GYM 
from operator import ne
from this import d
from turtle import end_fill, st
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete

# Import helpers
import os, sys
import numpy as np
from random import randint
from random import choice
import math

#Import SUMO
from tkinter.messagebox import NO
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

class CustomEnv(Env):
    def __init__(self, render=True):
        self.N_actions = 2
        self.action_space = Discrete(self.N_actions)
        self.n_adversaries = 2  # 2 going up and 2 going down
        self.n_features = 2     # distance to intersection and speed
        self.lanes = 2
        self.observation_space = Dict({'ego':Box(0,1,shape=(self.n_features,)), 'adversaries':Box(0,1,shape=(self.n_adversaries*self.n_features,)),})
        self.state = self.observation_space.sample()

        self.timestep = 0.1

        if render:
            sumo = "sumo-gui"
        else:
            sumo = "sumo"

        sumoCmd = [
            sumo,
            "-c",
            "sumo_config/roundabout.sumocfg",
            "--step-length",
            str(self.timestep),
            "--collision.action",
            "warn",
            "--collision.mingap-factor",
            "2",
            "--collision.check-junctions",
            "--random",
            "true",
            "--no-warnings",
        ]

        traci.start(sumoCmd)

        self.egoCarID = "ego"
        self.egoCar = True

        self.range = 50

        self.swap = False

        self.state = self.reset()

    def step(self, action):

        self._addCar()

        done = True
        for id in traci.vehicle.getIDList():
            if id == self.egoCarID:
                done = False
                break

        reward = 0
        if not done:            
            self._action(action)
            self.state = self._obs()
            self._collision()
            done, reward = self._reward(action)
        
        traci.simulationStep()

        info = {'time' : traci.simulation.getTime() - self.time_reset}

        return self.state, reward, done, info

    def reset(self):

        for id in traci.vehicle.getIDList():
            traci.vehicle.remove(id)
        traci.simulationStep()

        self.spawn_time = traci.simulation.getTime()

        self.time_car_added = traci.simulation.getTime()
        for i in range(200):
            self._addCar()
            traci.simulationStep()

        self.egoCar = False
        self._addCar()
        self.egoCar = True
        traci.simulationStep()

        # Reward parametres init
        self.collision = 0
        self.success = 0
        self.timeout = 0
        self.total_reward = 0

        self.state = self._obs()
        self.time_reset = traci.simulation.getTime()

        return self.state

    def _reward(self, action):
        done = False
        timeout = 200
        time = traci.simulation.getTime() - self.time_reset
        rt = 0
        x = traci.vehicle.getPosition(self.egoCarID)[0]
        v = traci.vehicle.getSpeed(self.egoCarID)

        if  time > timeout:
            self.timeout = 1
            done = True
        if  traci.vehicle.getRoadID(self.egoCarID) == "edge-north-SN":
            self.success = 1
            done = True
        if self.collision:
            done = True
            
        ks = 1
        kc = -2
        kt = -0.2
        kv = 0.0002
        # if done:
        #     rt = (kt * time) / timeout
        reward = ks * self.success + \
                 kc * self.collision + \
                 rt 
                #  kv * v 
                  

        self.total_reward += reward
        if done: 
            print("total reward: ",self.total_reward)  
                 

        return done, reward

    def _collision(self):
        if self.egoCarID in traci.simulation.getCollidingVehiclesIDList():
            self.collision = True
            print(traci.simulation.getCollidingVehiclesIDList())
        pass

    def _action(self, action):
        # set actions of ego-vehicle
        if action == 1:  #drive
            traci.vehicle.setSpeedMode(self.egoCarID, int('011111',2))
            traci.vehicle.setSpeed(self.egoCarID, 8.5)
        elif action == 0:   #stop
            traci.vehicle.setSpeedMode(self.egoCarID, int('100000',2))
            traci.vehicle.setSpeed(self.egoCarID, 0)
        pass
    
    def _obs(self):
        self.state["adversaries"] = np.array([1] * 4)
        self.state["ego"] = np.array([1] * 2)

        lane_length = traci.lane.getLength("edge-west-WS_0")

        d_max = 100
        v_max = 8.5
        adversaries_d = []
        adversaries_v = []


        for id in traci.vehicle.getIDList():
            if id != self.egoCarID and traci.vehicle.getRoadID(id) == "edge-west-WS":
                adversaries_d.append((lane_length - traci.vehicle.getLanePosition(id)) / d_max)
                adversaries_v.append((traci.vehicle.getSpeed(id)) / v_max)

        for _ in range(4):
            adversaries_d.append(1)
            adversaries_v.append(1)

        adversaries_v = [x for _, x in sorted(zip(adversaries_d, adversaries_v))]
        adversaries_d = np.sort(adversaries_d)

        self.state["adversaries"] = np.array([adversaries_d[0],adversaries_v[0],adversaries_d[1],adversaries_v[1]])
        self.state["ego"] = np.array([1, 1])  

        return self.state

    def _addCar(self):
        if self.egoCar and traci.simulation.getTime().is_integer():
            car = ["vType1", "vType2", "vType3", "vType4", "vType5"]
            routeUp = ["routeAdv1"]
            routeDown = ["routeAdv2"]
            if (traci.simulation.getTime() - self.time_car_added) > randint(4,8):   # 0 free, 1 traffic lights, 2 stops
                #Start a vehicle
                if self.lanes == 2:
                    idu = str(traci.simulation.getTime()) + "up"
                idd = str(traci.simulation.getTime()) + "down"

                if self.lanes == 2:
                    traci.vehicle.addFull(idu, choice(routeUp), depart=None, departPos='0', departSpeed='0', departLane='0', typeID=choice(car))
                    traci.vehicle.setSpeedMode(idu, int('101111',2))  
                traci.vehicle.addFull(idd, choice(routeDown), depart=None, departPos='0', departSpeed='0', departLane='0', typeID=choice(car))
                traci.vehicle.setSpeedMode(idd, int('101111',2))   

                
                self.time_car_added = traci.simulation.getTime()
        elif not self.egoCar:
            #Start the ego vehicle 
            traci.vehicle.addFull(self.egoCarID, 'routeEgo', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType0')
            traci.vehicle.setSpeedMode(self.egoCarID, int('100000',2))
            traci.vehicle.setSpeed(self.egoCarID, 0)
        pass
