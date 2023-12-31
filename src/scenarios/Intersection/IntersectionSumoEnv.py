# Import GYM 
from turtle import end_fill, st
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete

# Import helpers
import os, sys
import numpy as np
from random import randint

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
        self.observation_space = Box(0,1,shape=(6,))
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
            "3.5",
            "--collision.check-junctions",
            "--random",
            "true",
            "--no-warnings",
        ]

        traci.start(sumoCmd)

        self.egoCarID = "ego"
        self.egoCar = True

        self.state = self.reset()

    def step(self, action):

        if traci.simulation.getTime().is_integer() and self._free_spawn():
            self._addCar()

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
            done, reward = self._reward(action)

        traci.simulationStep()

        info = {}

        return self.state, reward, done, info


    def reset(self):

        for id in traci.vehicle.getIDList():
            traci.vehicle.remove(id)
        traci.simulationStep()

        # Intersection type
        self.intersection = randint(0,2)    # 0 free, 1 traffic lights, 2 stops
        if self.intersection == 0:
            traci.trafficlight.setProgram('0',0)
        elif self.intersection == 1:
            traci.trafficlight.setProgram('0',1)
            traci.trafficlight.setPhase('0',randint(0,3))
        elif self.intersection == 2:
            traci.trafficlight.setProgram('0',0)

        self.time_car_added = traci.simulation.getTime()
        for i in range(200):
            if traci.simulation.getTime().is_integer() and self._free_spawn():
                self._addCar()
            traci.simulationStep()

        self.egoCar = False
        self._addCar()
        self.egoCar = True
        traci.simulationStep()
        self.state = self._observation()
        self.time_reset = traci.simulation.getTime()

        # Reward parametres init
        self.collision = 0
        self.success = 0
        self.timeout = 0

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
        if  x > 540:
            self.success = 1
            done = True
        if self.collision:
            done = True
            
        ks = 1
        kc = -2
        kt = -0.2
        kv = 0.0002
        if done:
            rt = (kt * time) / timeout
        reward = ks * self.success + \
                 kc * self.collision + \
                 kv * v + \
                 rt
                 

        return done, reward

    def _addCar(self):
        if self.egoCar:
            if (traci.simulation.getTime() - self.time_car_added) > randint(4,8):   # 0 free, 1 traffic lights, 2 stops
                #Start a vehicle
                idu = str(traci.simulation.getTime()) + "up"
                idd = str(traci.simulation.getTime()) + "down"
                if self.intersection == 0:
                    traci.vehicle.addFull(idu, 'routeAdvUp', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType1')
                    traci.vehicle.setSpeedMode(idu, int('011111',2))  
                    traci.vehicle.addFull(idd, 'routeAdvDown', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType1')
                    traci.vehicle.setSpeedMode(idd, int('011111',2))   
                elif self.intersection == 1:
                    traci.vehicle.addFull(idu, 'routeAdvUp', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType2')
                    traci.vehicle.setSpeedMode(idu, int('011111',2))
                    traci.vehicle.addFull(idd, 'routeAdvDown', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType2')
                    traci.vehicle.setSpeedMode(idd, int('011111',2))
                elif self.intersection == 2:
                    traci.vehicle.addFull(idu, 'routeAdvUp', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType3')
                    traci.vehicle.setSpeedMode(idu, int('110110',2))
                    traci.vehicle.addFull(idd, 'routeAdvDown', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType3')
                    traci.vehicle.setSpeedMode(idd, int('110110',2))
                
                traci.vehicle.setSpeed(idu, randint(3,5))  
                traci.vehicle.setSpeed(idd, randint(3,5))
                self.time_car_added = traci.simulation.getTime()
        else:
            #Start the ego vehicle 
            traci.vehicle.addFull(self.egoCarID, 'routeEgo', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType0')
            traci.vehicle.setSpeedMode(self.egoCarID, int('00000',2))
            traci.vehicle.setSpeed(self.egoCarID, 0)
        pass

    def _observation(self):
        self.state = np.array([1] * 6)
        d_up=v_up=d_down=v_down=v_ego=d_ego=1

        for id in traci.vehicle.getIDList():
            if id[-1] == "p": #vehicle going up
                id_dist_up = (traci.vehicle.getPosition(id)[1] - 508.35) / 100
                if id_dist_up < d_up and id_dist_up > 0 and id != self.egoCarID:
                    d_up = id_dist_up
                    v_up = traci.vehicle.getSpeed(id) / 5

            if id[-1] == "n": #vehicle going down
                id_dist_down = (traci.vehicle.getPosition(id)[1] - 508.35) / 100
                if id_dist_down < d_down and id_dist_down > 0 and id != self.egoCarID:
                    d_down = id_dist_down
                    v_down = traci.vehicle.getSpeed(id) / 5

            if id == self.egoCarID:
                # ego vehicle velocity
                v_ego = traci.vehicle.getSpeed(self.egoCarID) / 5

                # distance to junction
                d_ego = (traci.junction.getPosition('51')[1] - traci.vehicle.getPosition(self.egoCarID)[0] )/ traci.junction.getPosition('51')[1]
                if d_ego < 0:
                    d_ego = 1
        
        
        self.state = np.array([v_ego, d_ego,d_down, v_down, d_up, v_up])
        

        return self.state

    def _action(self, action):
        # set actions of ego-vehicle
        if action == 0:  #drive
            traci.vehicle.setSpeedMode(self.egoCarID, int('00000',2))
            traci.vehicle.setSpeed(self.egoCarID, 5)
        elif action == 1:   #stop
            traci.vehicle.setSpeedMode(self.egoCarID, int('00000',2))
            traci.vehicle.setSpeed(self.egoCarID, 0)
        pass

    def _collision(self):
        if self.egoCarID in traci.simulation.getCollidingVehiclesIDList():
            self.collision = True
            # print("COLLISION")
        pass

    def _free_spawn(self):
        # for id in traci.vehicle.getIDList():
        #     if traci.vehicle.getRoadID(id) == "54o":
        #         return False
        return True
        



