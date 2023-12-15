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
		self.obs_space_ego = 2
		self.obs_space_adver = 6
		self.max_range = 100
		self.observation_space = Dict({'ego':Box(0,1,shape=(self.obs_space_ego,)), 'adversaries':Box(0,1,shape=(self.obs_space_adver,)),})
		self.state = self.observation_space.sample()
		self.timestep = 0.1
		self._prev_adver_bottom = []
		self._prev_adver_center = []
		self._prev_adver_top = []
		self.k_overtake = 0
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
			"1", 
			"--random",
			"true",
			"--no-warnings",
		]

		traci.start(sumoCmd)
		self.egoCarID = "ego"
		self.state = self.reset()

	def adversaries(self):

		for i in range(15):
			try:
				if str(i) in traci.vehicle.getIDList():
					traci.vehicle.remove(str(i))
			except:
				pass
			id_car = str(i)
			random_space = np.random.randint(7, 30)
			traci.vehicle.addFull(id_car, 'routeEgo', depart=None, departPos=str(i*45+random_space), departSpeed='0', departLane='random', typeID='vType1')
			traci.vehicle.setSpeedMode(id_car, int('00000',2))
			traci.vehicle.setSpeed(id_car, 2)
			traci.vehicle.setLaneChangeMode(id_car, 0)
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

	def _lane_reward(self):
		k_lane = 0.1
		lane_ego = traci.vehicle.getLaneIndex(self.egoCarID)
		if lane_ego == 0: #bottom
			k_lane = 0.3
		elif lane_ego == 1: #center
			k_lane = 0.2
		elif lane_ego == 2: #top
			k_lane = 0.1
		return k_lane

	def _calculate_best_trajectory(self, amount):
		reward = 0
		obs = self._get_vehicles_obs()
		lane_ego = traci.vehicle.getLaneIndex(self.egoCarID)
		dis_inf = obs[4]
		dis_cent = obs[2]
		if lane_ego == 0: # ego is in the bottom lane
			if dis_inf >= 0.9:
				reward = 0
			else :
				reward = amount
		elif lane_ego == 1: # ego is in the center lane
			if dis_inf > 0.8:
				reward = amount
			else:
				reward = 0
		'''elif lane_ego == 2: # ego is in the top lane
			if dis_inf > 0.8 and dis_cent > 0.8:
				reward = amount
			else:
				reward = 0'''
		return reward

	def _reward(self):
		reward = 0
		done = False
		timeout = 155 # 137 es lo maximo que deberia tardar

		time = traci.simulation.getTime() - self.time_reset

		if  time > timeout:
			self.timeout = 1
			reward = -6000
			done = True
		x_ego, _ = self._coords_ego()
		if  x_ego > 795:
			self.success = 1
			done = True
		if self.collision:
			reward  = -6000
			done = True
			
		if not done:
			v = self._calculate_best_trajectory(traci.vehicle.getSpeed(self.egoCarID))
			if v == 0 and self.k_overtake != 0:
				v = traci.vehicle.getSpeed(self.egoCarID)
			reward = round(v * (1 + self.k_overtake), 2)
			#print(f"reward: {reward}")

		self.total_reward += reward
		if done:
			self.time_reset = traci.simulation.getTime()

		return done, reward

	def _addEgoCar(self):
		# delay allows car to distribute
		for i in range(np.random.randint(10,20)):
			traci.simulationStep()
		
		#Start the ego vehicle 
		traci.vehicle.addFull(self.egoCarID, 'routeEgo', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType0')
		traci.vehicle.setSpeedMode(self.egoCarID, int('111111',2))
		traci.vehicle.setSpeed(self.egoCarID, 6)
		traci.vehicle.setLaneChangeMode(self.egoCarID, 0)

	def vehicle_in_range(self, id):
		if math.sqrt(pow(traci.vehicle.getPosition(self.egoCarID)[0] - traci.vehicle.getPosition(id)[0] ,2) + 
					 pow(traci.vehicle.getPosition(self.egoCarID)[1] - traci.vehicle.getPosition(id)[1] ,2)) < self.max_range:
			return True; return False

	def _get_vehicles_in_range(self):
		vehicles_in_range = []
		for id in traci.vehicle.getIDList():
			if self.vehicle_in_range(id) and id != self.egoCarID:
				x_adv = round(traci.vehicle.getPosition(id)[0], 2)
				lane_adv = traci.vehicle.getLaneIndex(id)
				v_adv = round(traci.vehicle.getSpeed(id), 1)
				vehicles_in_range.append([id, x_adv, lane_adv, v_adv])
		return vehicles_in_range

	def _round_number(self, number):
		if number != int(number) and number != 1.0 and number != 0.0:
			return round(number, 1)
		return number

	def _get_vehicles_obs(self):
		d_ll=d_cl=d_rl= 0
		v_ll=v_cl=v_rl= 0
		v_max = 6

		if self.egoCarID in traci.vehicle.getIDList():
			adver_in_range = self._get_vehicles_in_range()
			adver_bottom = []
			adver_top = []
			adver_center = []
			x_ego, _ = self._coords_ego()
			for i in range(len(adver_in_range)):
				x_adv = adver_in_range[i][1] + 8
				lane_adv = adver_in_range[i][2]
				if x_adv > x_ego: # in front
					if lane_adv == 0:
						adver_bottom.append(adver_in_range[i])
					if lane_adv == 1:
						adver_center.append(adver_in_range[i])
					if lane_adv == 2:
						adver_top.append(adver_in_range[i])
			
			#Order by distance
			adver_bottom.sort(key=lambda x: x[1])
			adver_center.sort(key=lambda x: x[1])
			adver_top.sort(key=lambda x: x[1])

			#Check if overtaking has been done
			if self._prev_adver_bottom != [] or self._prev_adver_center != [] or self._prev_adver_top != []:
				if (self._prev_adver_bottom != [] and adver_bottom == []) or (self._prev_adver_bottom != [] and adver_bottom != [] and adver_bottom[0][0] != self._prev_adver_bottom[0][0] and self._prev_adver_bottom[0][0] in traci.vehicle.getIDList()) \
					or (self._prev_adver_center != [] and adver_center == []) or (self._prev_adver_center != [] and adver_center != [] and adver_center[0][0] != self._prev_adver_center[0][0] and self._prev_adver_center[0][0] in traci.vehicle.getIDList()) \
					or (self._prev_adver_top != [] and adver_top == []) or (self._prev_adver_top != [] and adver_top != [] and adver_top[0][0] != self._prev_adver_top[0][0] and self._prev_adver_top[0][0] in traci.vehicle.getIDList()):
					self.k_overtake = 10
				else:
					self.k_overtake = 0

			#Store the previous values for reward
			self._prev_adver_bottom = adver_bottom
			self._prev_adver_center = adver_center
			self._prev_adver_top = adver_top
			
			if adver_top != []:
				d_rl = 1 - abs((adver_top[0][1] - x_ego) / self.max_range)
				v_rl = adver_top[0][3] / v_max
			if adver_center != []:
				d_ll = 1 - abs((adver_center[0][1] - x_ego) / self.max_range)
				v_ll = adver_center[0][3] / v_max
			if adver_bottom != []:
				d_cl =  1 - abs((adver_bottom[0][1] - x_ego) / self.max_range)
				v_cl = adver_bottom[0][3] / v_max
			
		res = np.array([d_rl, v_rl,d_ll, v_ll, d_cl, v_cl,])
		for i in range(len(res)):
			res[i] = self._round_number(res[i])
		return res

	def _observation(self):
		v_max = 6
		v_ego = 1
		lane = -1
		self.state["adversaries"] = np.array(self.obs_space_adver)
		self.state["ego"] = np.array(self.obs_space_ego)
		if self.egoCarID in traci.vehicle.getIDList():
			self.state["adversaries"] = self._get_vehicles_obs()
			#print(self.state["adversaries"])

			lane = (traci.vehicle.getLaneIndex(self.egoCarID) + 1)  # 1, 2, 3
			v_ego = traci.vehicle.getSpeed(self.egoCarID) / v_max
			self.state["ego"] = np.array([lane, v_ego])
			#print(self.state["ego"])

		return self.state

	def _action(self, action):
		index = traci.vehicle.getLaneIndex(self.egoCarID)
		if action == 0 and index > 0: #Change Right
			index -= 1
			traci.vehicle.changeLane(self.egoCarID, index, 0)

		
		if action == 2 and index < 2: #Change Left
			index += 1
			traci.vehicle.changeLane(self.egoCarID, index, 0)

	def _collision(self):
		if self.egoCarID in traci.simulation.getCollidingVehiclesIDList():
			self.collision = 1
		pass




