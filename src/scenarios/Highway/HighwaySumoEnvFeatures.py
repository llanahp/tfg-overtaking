# Import GYM 
from turtle import end_fill, st
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete
import uuid
from colorama import Fore, Back, Style, init
from math import log
from operator import ne


# Import helpers
import os, sys
import numpy as np
from random import randint, choice
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
		self.action_space = Discrete(3) #Box(low=0, high=1, shape=(1,))
		self.N_actions = 3
		self.action_space = Discrete(self.N_actions)
		self.observation_space = Dict({'ego':Box(0,1,shape=(2,)), 'adversaries':Box(0,1,shape=(12,)),})
		self.state = self.observation_space.sample()
		self.timestep = 0.1
		sumo = "sumo-gui" if render else "sumo"

		sumoCmd = [
			sumo,
			"-c",
			"scenarios/Highway/sumo_config/highway.sumocfg",
			"--step-length",
			str(self.timestep),
			"--collision.action",
			"warn",
			"--collision.mingap-factor",
			"1", # 2 in overtaking env
			"--random",
			"true",
			"--no-warnings",
		]

		traci.start(sumoCmd)

		self.egoCarID = "ego"

		self.state = self.reset()

	def adversaries(self):

		for i in range(25):
			try:
				if str(i) in traci.vehicle.getIDList():
					traci.vehicle.remove(str(i))
			except:
				pass
			id_car = str(i)
			traci.vehicle.addFull(id_car, 'routeEgo', depart=None, departPos=str(i*30+30), departSpeed='0', departLane='random', typeID='vType1')
			traci.vehicle.setSpeedMode(id_car, int('00000',2))
			traci.vehicle.setSpeed(id_car, 3)
			traci.vehicle.setLaneChangeMode(id_car, int('00000',2))
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

		if done:
			print(f"total reward: {self.total_reward}")
			self.time_reset = traci.simulation.getTime()
			self.reset()

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

		# Reward parametres init
		self.collision = 0
		self.success = 0
		self.timeout = 0
		self.total_reward = 0

		self.time_reset = traci.simulation.getTime()
		self.success = False
		self.total_reward = 0

		return self.state

	def _coords_ego(self):
		x_ego = traci.vehicle.getPosition(self.egoCarID)[0]
		y_ego = traci.vehicle.getPosition(self.egoCarID)[1]
		return x_ego, y_ego
	
	def _reward(self):
		reward = 0
		done = False
		timeout = 210
		

		ks = 1
		kc = -2
		kv = 0.0001 
		ki = 0
		time = traci.simulation.getTime() - self.time_reset

		'''if  time > timeout:
			self.timeout = 1
			done = True'''

		x_ego, y_ego = self._coords_ego()
		v = traci.vehicle.getSpeed(self.egoCarID)

		if  x_ego > 795:
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
			#print("total reward: ",self.total_reward)
			self.time_reset = traci.simulation.getTime()

		return done, reward

	def _addEgoCar(self):
		# delay allows car to distribute
		for i in range(np.random.randint(40,50)):
			traci.simulationStep()
		
		#Start the ego vehicle 
		traci.vehicle.addFull(self.egoCarID, 'routeEgo', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType0')
		#traci.vehicle.setLaneChangeMode(self.egoCarID, int('00000',2))
		traci.vehicle.setSpeedMode(self.egoCarID, int('111111',2))
		traci.vehicle.setSpeed(self.egoCarID, 6)
		traci.vehicle.setLaneChangeMode(self.egoCarID, 0)

	def _observation(self):
		d_ll=d_cl=d_rl=d_lf=d_cf=d_rf = 1
		v_ll=v_cl=v_rl=v_lf=v_cf=v_rf = 0
		v_ego = 1
		lane = -1
		max_range = 100
		v_max = 6
		self.state["adversaries"] = np.array([1] * 12)
		self.state["ego"] = np.array([2])

		if self.egoCarID in traci.vehicle.getIDList():
			if traci.vehicle.getLeftLeaders(self.egoCarID): # if there is a car on the left lane
				id_oponent = traci.vehicle.getLeftLeaders(self.egoCarID)[0][0]
				#print("1.- id:", id_oponent)
				d_ll = traci.vehicle.getLeftLeaders(self.egoCarID)[0][1] / max_range
				v_ll = traci.vehicle.getSpeed(traci.vehicle.getLeftLeaders(self.egoCarID)[0][0]) / v_max
			if traci.vehicle.getLeader(self.egoCarID): # if there is a car on the same lane
				#print("2.- id:", traci.vehicle.getLeader(self.egoCarID)[0])
				d_cl = traci.vehicle.getLeader(self.egoCarID)[1] / max_range
				v_cl = traci.vehicle.getSpeed(traci.vehicle.getLeader(self.egoCarID)[0]) / v_max
			if traci.vehicle.getRightLeaders(self.egoCarID): # if there is a car on the right lane
				#print("3.- id:", traci.vehicle.getRightLeaders(self.egoCarID)[0][0])
				d_rl = traci.vehicle.getRightLeaders(self.egoCarID)[0][1] / max_range
				v_rl = traci.vehicle.getSpeed(traci.vehicle.getRightLeaders(self.egoCarID)[0][0]) / v_max
			if traci.vehicle.getLeftFollowers(self.egoCarID):
				#print("4.- id:", traci.vehicle.getLeftFollowers(self.egoCarID)[0][0])
				d_lf = traci.vehicle.getLeftFollowers(self.egoCarID)[0][1] / max_range
				v_lf = traci.vehicle.getSpeed(traci.vehicle.getLeftFollowers(self.egoCarID)[0][0]) / v_max
			if traci.vehicle.getFollower(self.egoCarID)[1]!=-1:
				#print("5.- id:", traci.vehicle.getFollower(self.egoCarID)[0])
				d_cf = traci.vehicle.getFollower(self.egoCarID)[1] / max_range
				v_cf = traci.vehicle.getSpeed(traci.vehicle.getFollower(self.egoCarID)[0]) / v_max
			if traci.vehicle.getRightFollowers(self.egoCarID):
				#print("6.- id:", traci.vehicle.getRightFollowers(self.egoCarID)[0][0])
				d_rf = traci.vehicle.getRightFollowers(self.egoCarID)[0][1] / max_range
				v_rf = traci.vehicle.getSpeed(traci.vehicle.getRightFollowers(self.egoCarID)[0][0]) / v_max
			self.state["adversaries"] = np.array([d_ll, v_ll, d_cl, v_cl, d_rl, v_rl, d_lf, v_lf, d_cf, v_cf, d_rf, v_rf])
			

			for i in range(len(self.state["adversaries"])):
				if self.state["adversaries"][i] != int(self.state["adversaries"][i]) and self.state["adversaries"][i] != 1.0 and self.state["adversaries"][i] != 0.0:
					self.state["adversaries"][i] = round(self.state["adversaries"][i], 1)

			#print(self.state["adversaries"])
			
			if self.egoCarID in traci.vehicle.getIDList():
				lane = traci.vehicle.getLaneIndex(self.egoCarID)
			v_ego = traci.vehicle.getSpeed(self.egoCarID) / v_max
			self.state["ego"] = np.array([lane, v_ego])

		return self.state

	def _action(self, action):
		'''
			action 0: change left
			action 1: keep lane
			action 2: change right
		'''
		index = traci.vehicle.getLaneIndex(self.egoCarID)
		
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




