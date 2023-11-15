# Import GYM 
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete

# Import helpers
import os, sys
import numpy as np
from random import randint
import copy

# Imports Prediction Modules
from scenarios.SMARTS.prediction.predictions_utils import Motion_Predictor

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
        self.n_vehicles = 11
        self.n_states = 4
        self.n_obs = 50

        self.observation_space = Box(0,1,shape=(self.n_vehicles, self.n_states, self.n_features), dtype=np.float32)
        self.empty = self.observation_space.sample()
        self.list_of_ids = {}

        for num_adv in range(self.n_vehicles):
            self.empty[num_adv] = np.zeros((self.n_states, self.n_features))

        self.timestep = 0.1
        self.timestamp = 0
        self.render = render
        self._run_sumo()

        self.egoCarID = "ego"
        self.adversaries_keys = dict()
        self.adversaries_id_cnt = 1

        self.motion_predictor = Motion_Predictor()

        self.write_csv = False
        self.predict = True

    def step(self, action):

        self.timestamp += 1

        self._action(action)
        state = self._obs()
        done, reward = self._reward(action)
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
        for _ in range(randint(40,60)):
            traci.simulationStep()

        self._addCar()

        # Reward parametres init
        self.collision = 0
        self.success = 0
        self.timeout = 0
        self.total_reward = 0

        state = self._obs()
        self.time_reset = traci.simulation.getTime()
        self.timestamp = 0

        self.list_of_ids.clear()
        self.adversaries_keys.clear()
        self.adversaries_id_cnt = 1

        return state

    def _reward(self, action):
        reward = 0
        done = False
        timeout = 30
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
            kt = -1
            reward = ks * self.success + \
                     kc * self.collision + \
                     kt * self.timeout
        else:
            kv = 0.0005
            reward = kv * v
                  
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
        v = traci.vehicle.getSpeed(self.egoCarID)
        if action == 1:  # increase vel
            v = v + 1
            if v > 10: v = 10 
        elif action == 0:   # reduce vel
            v = v - 1
            if v < 0: v = 0 

        traci.vehicle.setSpeedMode(self.egoCarID, int('100000',2))
        traci.vehicle.setSpeed(self.egoCarID, v)
        traci.simulationStep()
        self._collision()
    
    def _obs(self):
        state = copy.deepcopy(self.empty)
        obs = {}

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

        if "ego" in self.list_of_ids:
            self.list_of_ids["ego"].append([traci.vehicle.getPosition(self.egoCarID)[0], traci.vehicle.getPosition(self.egoCarID)[1], 1])
        else:
            self.list_of_ids["ego"] = [[0, 0, 0] for _ in range(self.n_obs)]
            self.list_of_ids["ego"].append([traci.vehicle.getPosition(self.egoCarID)[0], traci.vehicle.getPosition(self.egoCarID)[1], 1])

        for adv in range(len(adversaries)):
            adv_id = adversaries[adv][0]
            x_adv = adversaries[adv][1]
            y_adv = adversaries[adv][2]

            if adv_id in self.list_of_ids:
                self.list_of_ids[adv_id].append([x_adv, y_adv, 1])
            else:
                self.list_of_ids[adv_id] = [[0, 0, 0] for _ in range(self.n_obs)]
                self.list_of_ids[adv_id].append([x_adv, y_adv, 1])

        # n_adv = min(len(adversaries), self.n_vehicles)
        for adv in range(len(adversaries)+1):
            # current_adv = self.list_of_keys[adv]
            if adv == 0:
                obs[0] = np.array(self.list_of_ids["ego"][-50:])
            else:
                obs[adversaries[adv-1][0]] = np.array(self.list_of_ids[adversaries[adv-1][0]][-50:])

        if self.write_csv: self.motion_predictor.write_csv(obs, self.timestamp)

        if self.predict: 
            valid_agents_info, valid_agents_id = self.motion_predictor.preprocess_trackers(obs)
            if valid_agents_info: # Agents with more than a certain number of observations
                predictions, confidences = self.motion_predictor.predict_agents(valid_agents_info, self.timestamp)
                for num_adv in range(self.n_vehicles):
                    if num_adv < len(predictions)-1:
                        state[num_adv] = predictions[num_adv]
                    else:
                        break
        else:
            keys_obs = [key for key in obs.keys()]
            for num_adv in range(self.n_vehicles):
                key = keys_obs[num_adv]
                if num_adv < len(obs)-1:
                    state[num_adv][0] = [obs[key][49][0], obs[key][49][1]]
                else:
                    break

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

