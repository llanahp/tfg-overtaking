# Import GYM 
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete

# Import helpers
import os, sys
import numpy as np
from random import randint
import copy

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
        self.N_actions = 2
        self.action_space = Discrete(self.N_actions)
        self.n_features = 2
        self.n_vehicles = 6
        self.n_states = 1

        self.observation_space = Dict({'ego':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle A':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle B':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle C':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle D':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle E':Box(0,1,shape=(self.n_states, self.n_features))})

        self.list_of_keys = [key for key in self.observation_space.keys()]
        self.empty = self.observation_space.sample()

        for num_adv in range(self.n_vehicles):
            current_adv = self.list_of_keys[num_adv]
            self.empty[current_adv] = np.zeros((self.n_states, self.n_features))

        self.timestep = 0.1
        self.render = render
        self._run_sumo()

        self.egoCarID = "ego"
        self.adversaries_keys = dict()
        self.adversaries_id_cnt = 1

    def step(self, action):

        self.timestamp += 1

        self._action(action)
        state = self._obs()
        done, reward = self._reward(action)

        if self.force_action:
            while not done:
                self._collision()
                done, reward = self._reward(action)
                traci.simulationStep()

        info = {'time' : traci.simulation.getTime() - self.time_reset}

        return state, reward, done, info

    def reset(self):
        if traci.simulation.getMinExpectedNumber() == 1:
            traci.close()
            self._run_sumo()

        # Remove vehicles from the scenario
        for id in traci.vehicle.getIDList():
            traci.vehicle.remove(id)
        traci.simulationStep()

        # Wait 4 to 6 seconds to create the ego vehicle 
        for _ in range(randint(10,40)):
            traci.simulationStep()

        self._addCar()

        # Reward parametres init
        self.collision = 0
        self.success = 0
        self.timeout = 0
        self.total_reward = 0

        self.time_reset = traci.simulation.getTime()
        self.timestamp = 0

        self.adversaries_keys.clear()
        self.adversaries_id_cnt = 1

        # Actions
        self.force_action = False

        self._reach_intersection()
        state = self._obs()

        return state

    def _reward(self, action):
        reward = 0
        done = False
        timeout = 20
        time = traci.simulation.getTime() - self.time_reset
        x = traci.vehicle.getPosition(self.egoCarID)[0]
        v = traci.vehicle.getSpeed(self.egoCarID)

        if  time > timeout:
            self.timeout = 1
            done = True
        if  x < -40 and x > -50:
            self.success = True
            done = True
        if self.collision:
            done = True
        
        if done:
            ks = 1
            kc = -1
            reward = ks * self.success + \
                     kc * self.collision
        # else:
        #     kv = 0.0005
        #     reward = kv * v
                  
        self.total_reward += reward

        if done: 
            print("total reward: ",self.total_reward)  
                 
        return done, reward

    def _collision(self):
        if self.egoCarID in traci.simulation.getCollidingVehiclesIDList():
            self.collision = True
        pass

    def _action(self, action):
        # set actions of ego-vehicle
        if action == 1:  #drive
            traci.vehicle.setSpeedMode(self.egoCarID, int('100000',2))
            traci.vehicle.setSpeed(self.egoCarID, 10)
            self.force_action = True
        elif action == 0:   #stop
            traci.vehicle.setSpeedMode(self.egoCarID, int('100000',2))
            traci.vehicle.setSpeed(self.egoCarID, 0)
        pass

        # # set actions of ego-vehicle
        # v = traci.vehicle.getSpeed(self.egoCarID)
        # if action == 1:  # increase vel
        #     v = v + 1
        #     if v > 10: v = 10 
        # elif action == 0:   # reduce vel
        #     v = v - 1
        #     if v < 0: v = 0 

        # traci.vehicle.setSpeedMode(self.egoCarID, int('100000',2))
        # traci.vehicle.setSpeed(self.egoCarID, v)
        traci.simulationStep()
        self._collision()
    
    def _obs(self):
        state = copy.deepcopy(self.empty)

        if self.egoCarID in traci.vehicle.getIDList():
            pass
        else:
            return state

        adversaries = []
        for id in traci.vehicle.getIDList():
            if id != self.egoCarID:
                current_id = id.split("--")[-1]
                if not current_id in self.adversaries_keys.keys():
                    self.adversaries_keys[current_id] = self.adversaries_id_cnt
                    self.adversaries_id_cnt += 1

                adversaries.append([self.adversaries_keys[current_id], traci.vehicle.getPosition(id)[0], traci.vehicle.getPosition(id)[1]])

        while(len(adversaries)<10):
            adversaries.append([0, 0, 0])

        for num_adv in range(self.n_vehicles):
            current_adv = self.list_of_keys[num_adv]
            if num_adv == 0:
                state[current_adv] = np.array([[traci.vehicle.getPosition(self.egoCarID)[0], traci.vehicle.getPosition(self.egoCarID)[1]]])
            else:
                state[current_adv] = np.array([[adversaries[num_adv][1], adversaries[num_adv][2]]])

        return state


    def _addCar(self):
        # Remove vehicles from the scenario
        while self.egoCarID in traci.vehicle.getIDList():
            traci.vehicle.remove(self.egoCarID)

        #Start the ego vehicle 
        traci.vehicle.addFull(self.egoCarID, 'routeEgo', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType0')
        traci.vehicle.setSpeedMode(self.egoCarID, int('100000',2))
        traci.vehicle.setSpeed(self.egoCarID, 10)
        traci.simulationStep()

        while not self.egoCarID in traci.vehicle.getIDList():
            traci.simulationStep()

    def _reach_intersection(self):
        while(traci.vehicle.getPosition(self.egoCarID)[1] < 87):
            traci.vehicle.setSpeedMode(self.egoCarID, int('01111',2))
            traci.vehicle.setSpeed(self.egoCarID, 10)
            traci.simulationStep()

        traci.vehicle.setSpeedMode(self.egoCarID, int('00000',2))
        traci.vehicle.setSpeed(self.egoCarID, 0)
        traci.simulationStep()


    def _run_sumo(self):
        if self.render:
            sumo = "sumo-gui"
        else:
            sumo = "sumo"

        sumoCmd = [
            sumo,
            "-c",
            "scenarios/SMARTS/sumo_config/1_to_2lane_left_turn_t_agents_1/scenario.sumocfg",
            "--step-length",
            str(self.timestep),
            "--collision.action",
            "warn",
            "--collision.mingap-factor",
            "1",
            "--collision.check-junctions",
            "--random",
            "true",
            "--no-warnings",
        ]

        traci.start(sumoCmd)
