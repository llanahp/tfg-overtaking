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
    def __init__(self, render=False):
        self.N_actions = 2
        self.action_space = Discrete(self.N_actions)
        self.n_adversaries = 4  # 2 going up and 2 going down
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
            "scenarios/Intersection/sumo_config/intersection.sumocfg",
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

        self.swap = True

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

        if self.force_action:
            while not done:
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

        # Intersection type

        self.intersection = 0#randint(0,2)    # 0 free, 1 traffic lights, 2 stops
        if self.intersection == 0:
            traci.trafficlight.setProgram('0',0)
        elif self.intersection == 1:
            traci.trafficlight.setProgram('0',1)
            traci.trafficlight.setPhase('0',randint(0,3))
        elif self.intersection == 2:
            traci.trafficlight.setProgram('0',0)

        self.swap = not self.swap
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

        # Actions
        self.force_action = False

        self._reach_intersection()

        self.state = self._obs()
        self.time_reset = traci.simulation.getTime()

        return self.state

    def _reach_intersection(self):
        while(traci.junction.getPosition('51')[1] - traci.vehicle.getPosition(self.egoCarID)[0]>10):
            traci.vehicle.setSpeedMode(self.egoCarID, int('00000',2))
            traci.vehicle.setSpeed(self.egoCarID, 8.5)
            traci.simulationStep()
            self._addCar()

        traci.vehicle.setSpeedMode(self.egoCarID, int('00000',2))
        traci.vehicle.setSpeed(self.egoCarID, 0)
        traci.simulationStep()

    def _reward(self, action):
        done = False
        timeout = 30
        time = traci.simulation.getTime() - self.time_reset
        rt = 0
        x = traci.vehicle.getPosition(self.egoCarID)[0]
        v = traci.vehicle.getSpeed(self.egoCarID)

        if  time > timeout:
            self.timeout = 1
            done = True
        if  x > 540:
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
            self.force_action = True
        elif action == 0:   #stop
            traci.vehicle.setSpeedMode(self.egoCarID, int('100000',2))
            traci.vehicle.setSpeed(self.egoCarID, 0)
        pass
    
    def _obs(self):
        self.state["adversaries"] = np.array([1] * 8)
        self.state["ego"] = np.array([1] * 2)

        d_max = 100
        v_max = 8.5
        adversaries_yaw = []
        adversaries_y = []
        adversaries_x = []
        adversaries_v = []
        d_up = []
        v_up = []
        d_down = []
        v_down = []
        v_ego = 1
        d_ego = 1

        for id in traci.vehicle.getIDList():
            if id != self.egoCarID and self.vehicle_in_range(id):
                sensor = 0 #randint(0,5) -2.5 
                x_adv, y_adv = self.local_to_global(id)
                adversaries_y.append(x_adv + sensor)
                adversaries_x.append(y_adv + sensor)
                adversaries_v.append(traci.vehicle.getSpeed(id))  
                adversaries_yaw.append(traci.vehicle.getAngle(self.egoCarID) - traci.vehicle.getAngle(id))
            
        if adversaries_yaw:
            for i in range(len(adversaries_yaw)):
                if adversaries_yaw[i] < -45 and adversaries_yaw[i] > -135:
                    d_down.append(round(adversaries_y[i],2) / d_max)
                    v_down.append(adversaries_v[i] / v_max) 
                    d_ego = self.collision_point(d_ego, adversaries_x[i], d_max)
                elif adversaries_yaw[i] > 45 and adversaries_yaw[i] < 135:
                    d_up.append(round(-adversaries_y[i],2) / d_max)
                    v_up.append(adversaries_v[i] / v_max)    
                    d_ego = self.collision_point(d_ego, adversaries_x[i], d_max) 

        for _ in range(8):
            d_up.append(1)
            d_down.append(1)
            v_up.append(1)
            v_down.append(1)
        
        d_up = np.sort(d_up)
        d_down = np.sort(d_down)

        #Get the positive values (closer distances to the intersection)
        d_up = d_up[d_up>0]
        d_down = d_down[d_down>0]

        v_up_i = np.array(v_up).size - d_up.size
        v_down_i = np.array(v_down).size - d_down.size

        self.state["adversaries"] = np.array([d_down[0],v_down[v_down_i],d_down[1],v_down[v_down_i+1],d_up[0],v_up[v_up_i],d_up[1],v_up[v_up_i+1]])
        self.state["ego"] = np.array([1, 1])  

        return self.state

    def collision_point(self, d_ego, d, d_max):
        new_d_ego = -d  / d_max

        if new_d_ego < 0:
            new_d_ego = 1

        if new_d_ego < d_ego:
            return new_d_ego
        else: 
            return d_ego


    def vehicle_in_range(self, id):
        if math.sqrt(pow(traci.vehicle.getPosition(self.egoCarID)[0] - traci.vehicle.getPosition(id)[0] ,2) + 
                     pow(traci.vehicle.getPosition(self.egoCarID)[1] - traci.vehicle.getPosition(id)[1] ,2)) < self.range:
                     return True; return False 

    def local_to_global(self, id):
        x_ego = traci.vehicle.getPosition(self.egoCarID)[0]
        y_ego = traci.vehicle.getPosition(self.egoCarID)[1]
        yaw = (traci.vehicle.getAngle(self.egoCarID) * math.pi) / 180
        x_adv = traci.vehicle.getPosition(id)[0]
        y_adv = traci.vehicle.getPosition(id)[1]
        x = (x_adv - x_ego)*math.cos(yaw) + (y_adv - y_ego)*math.sin(yaw)
        y = -(x_adv - x_ego)*math.sin(yaw) + (y_adv - y_ego)*math.cos(yaw)
        return x, y

    def _addCar(self):
        if self.egoCar and traci.simulation.getTime().is_integer() and self._free_spawn() and self.swap:
            car = ["vType1", "vType2", "vType3", "vType4", "vType5"]
            routeUp = ["routeAdvUp", "routeAdvUpRight", "routeAdvUpLeft"]
            routeDown = ["routeAdvDown", "routeAdvDownRight", "routeAdvDownLeft"]
            # routeUp = ["routeAdvUp"]
            # routeDown = ["routeAdvDown"]
            if (traci.simulation.getTime() - self.time_car_added) > randint(4,8):   # 0 free, 1 traffic lights, 2 stops
                #Start a vehicle
                if self.lanes == 2:
                    idu = str(traci.simulation.getTime()) + "up"
                idd = str(traci.simulation.getTime()) + "down"
                if self.intersection == 0:
                    if self.lanes == 2:
                        traci.vehicle.addFull(idu, choice(routeUp), depart=None, departPos='0', departSpeed='0', departLane='0', typeID=choice(car))
                        traci.vehicle.setSpeedMode(idu, int('101111',2))  
                    traci.vehicle.addFull(idd, choice(routeDown), depart=None, departPos='0', departSpeed='0', departLane='0', typeID=choice(car))
                    traci.vehicle.setSpeedMode(idd, int('101111',2))   
                elif self.intersection == 1:
                    if self.lanes == 2:
                        traci.vehicle.addFull(idu, choice(routeUp), depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType2')
                        traci.vehicle.setSpeedMode(idu, int('101111',2))
                    traci.vehicle.addFull(idd, choice(routeDown), depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType2')
                    traci.vehicle.setSpeedMode(idd, int('101111',2))
                elif self.intersection == 2:
                    if self.lanes == 2:
                        traci.vehicle.addFull(idu, choice(routeUp), depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType3')
                        traci.vehicle.setSpeedMode(idu, int('110111',2))
                    traci.vehicle.addFull(idd, choice(routeDown), depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType3')
                    traci.vehicle.setSpeedMode(idd, int('110111',2))
                
                self.time_car_added = traci.simulation.getTime()
        elif not self.egoCar:
            #Start the ego vehicle
            print("-----> adding ego car")
            traci.vehicle.addFull(self.egoCarID, 'routeEgo', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType0')
            traci.vehicle.setSpeedMode(self.egoCarID, int('100000',2))
            traci.vehicle.setSpeed(self.egoCarID, 0)
        pass

    def _free_spawn(self):
        # for id in traci.vehicle.getIDList():
        #     if traci.vehicle.getRoadID(id) == "54o":
        #         return False
        # if traci.simulation.getTime() - self.spawn_time > randint(1,5):
        if traci.simulation.getTime() - self.spawn_time > 0:
            self.spawn_time = traci.simulation.getTime()
            return True
        else:
            return False
