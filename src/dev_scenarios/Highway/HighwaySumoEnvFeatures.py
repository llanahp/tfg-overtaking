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

        if render:
            sumo = "sumo-gui"
        else:
            sumo = "sumo"

        sumoCmd = [
            sumo,
            "-c",
            "Highway/sumo_config/highway.sumocfg",
            "--step-length",
            str(0.1),
            "--collision.action",
            "warn",
            "--collision.mingap-factor",
            "1",
            "--random",
            "true",
            "--no-warnings",
        ]

        traci.start(sumoCmd)

        self.egoCarID = "ego"

        self.state = self.reset()

    def adversaries(self):

        for i in range(12):
            try:
                traci.vehicle.remove(str(i))
            except:
                pass
            traci.vehicle.addFull(str(i), 'routeEgo', depart=None, departPos=str(i*30+30), departSpeed='0', departLane='random', typeID='vType1')
            traci.vehicle.setSpeedMode(str(i), int('00000',2))
            traci.vehicle.setSpeed(str(i), 5)
            traci.vehicle.setLaneChangeMode(str(i), int('00000',2))
            traci.simulationStep()
            i+=1

    def step(self, action):
        done = True
        for id in traci.vehicle.getIDList():
            if id == self.egoCarID:
                done = False
                break

        reward = 0
        if not done:            
            self._action(action)
            self.state = self._observation()
            self._collision()
            done, reward = self._reward()

        for i in range(12):
            try:
                traci.vehicle.setSpeedMode(str(i), int('011111',2))
                new_speed = traci.vehicle.getSpeed(str(i)) + np.random.randint(0,5) - 2
                if new_speed  < 0: new_speed = 0
                traci.vehicle.setSpeed(str(i), 5)
                traci.vehicle.setLaneChangeMode(str(i), int('100000',2))
            except:
                pass
            i+=1

        traci.simulationStep()

        info = {'time' : traci.simulation.getTime() - self.time_reset}

        return self.state, reward, done, info


    def reset(self):

        self.adversaries()

        try:
            traci.vehicle.remove(self.egoCarID)
        except:
            pass
        
        self._addEgoCar()
        traci.simulationStep()
        self.state = self._observation()
        self.collision = 0

        self.time_reset = traci.simulation.getTime()
        self.success = False
        self.total_reward = 0

        return self.state

    
    def _reward(self):

        reward = 0
        done = False

        ks = 1
        kc = -2
        kv = 0.0001
        ki = 0

        x = traci.vehicle.getPosition(self.egoCarID)[0]
        v = traci.vehicle.getSpeed(self.egoCarID)

        if  x > 390:
            self.success = 1
            done = True
        if self.collision:
            done = True
        if traci.vehicle.getLaneIndex(self.egoCarID) == 0:
            ki = 0.001     

        reward = ks * self.success + \
                 kc * self.collision + \
                 kv * v + \
                 ki

        self.total_reward += reward
        if done: 
            print("total reward: ",self.total_reward)

        return done, reward

    def _addEgoCar(self):
        # delay allows car to distribute
        for i in range(np.random.randint(40,50)):
            traci.simulationStep()
        
        #Start the ego vehicle 
        traci.vehicle.addFull(self.egoCarID, 'routeEgo', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType0')
        traci.vehicle.setLaneChangeMode(self.egoCarID, int('00000',2))

    def _observation(self):
        d_ll=d_cl=d_rl=d_lf=d_cf=d_rf = 1
        v_ll=v_cl=v_rl=v_lf=v_cf=v_rf = 1
        v_ego = 1
        max_range = 100
        v_max = 30
        self.state["adversaries"] = np.array([1] * 12)
        self.state["ego"] = np.array([1])

        for id in traci.vehicle.getIDList():
            if id == self.egoCarID:
                if traci.vehicle.getLeftLeaders(self.egoCarID):
                    d_ll = traci.vehicle.getLeftLeaders(self.egoCarID)[0][1] / max_range
                    v_ll = traci.vehicle.getSpeed(traci.vehicle.getLeftLeaders(self.egoCarID)[0][0]) / v_max
                if traci.vehicle.getLeader(self.egoCarID):
                    d_cl = traci.vehicle.getLeader(self.egoCarID)[1] / max_range
                    v_cl = traci.vehicle.getSpeed(traci.vehicle.getLeader(self.egoCarID)[0]) / v_max
                if traci.vehicle.getRightLeaders(self.egoCarID):
                    d_rl = traci.vehicle.getRightLeaders(self.egoCarID)[0][1] / max_range
                    v_rl = traci.vehicle.getSpeed(traci.vehicle.getRightLeaders(self.egoCarID)[0][0]) / v_max
                if traci.vehicle.getLeftFollowers(self.egoCarID):
                    d_lf = traci.vehicle.getLeftFollowers(self.egoCarID)[0][1] / max_range
                    v_lf = traci.vehicle.getSpeed(traci.vehicle.getLeftFollowers(self.egoCarID)[0][0]) / v_max
                if traci.vehicle.getFollower(self.egoCarID)[1]!=-1:
                    d_cf = traci.vehicle.getFollower(self.egoCarID)[1] / max_range
                    v_cf = traci.vehicle.getSpeed(traci.vehicle.getFollower(self.egoCarID)[0]) / v_max
                if traci.vehicle.getRightFollowers(self.egoCarID):
                    d_rf = traci.vehicle.getRightFollowers(self.egoCarID)[0][1] / max_range
                    v_rf = traci.vehicle.getSpeed(traci.vehicle.getRightFollowers(self.egoCarID)[0][0]) / v_max
                self.state["adversaries"] = np.array([d_ll, v_ll, d_cl, v_cl, d_rl, v_rl, d_lf, v_lf, d_cf, v_cf, d_rf, v_rf])

                v_ego = traci.vehicle.getSpeed(self.egoCarID)  / v_max
                self.state["ego"] = np.array([v_ego])
                break

        return self.state

    def _action(self, action):

        index = traci.vehicle.getLaneIndex(self.egoCarID)
        traci.vehicle.setSpeedMode(self.egoCarID, int('011111',2))
        traci.vehicle.setSpeed(self.egoCarID, 25)
        
        if action == 0 and index < 2: #Change Left
            index += 1
            traci.vehicle.changeLane(self.egoCarID, index, 0)

        if action == 2 and index > 0: #Change Right
            index -= 1
            traci.vehicle.changeLane(self.egoCarID, index, 0)
    
    def _collision(self):
        if self.egoCarID in traci.simulation.getCollidingVehiclesIDList():
            self.collision = 1
        pass




