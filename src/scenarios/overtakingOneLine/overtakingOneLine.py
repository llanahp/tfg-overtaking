# Import GYM 
from turtle import end_fill, st
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete
import uuid
from colorama import Fore, Back, Style, init
from math import log
from operator import ne
import random


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
		self.obs_space_ego = 2
		self.obs_space_adver = 8
		self.max_range = 100
		self.observation_space = Dict({'ego':Box(0,1,shape=(self.obs_space_ego,)), 'adversaries':Box(0,1,shape=(self.obs_space_adver,)),})
		self.state = self.observation_space.sample()
		self.timestep = 0.1
		self.inside_cars = []
		sumo = "sumo-gui" if render else "sumo"

		sumoCmd = [
			sumo,
			"-c",
			"scenarios/overtakingOneLine/sumo_config/overtaking.sumocfg",
			"--step-length",
			str(self.timestep),
			"--collision.action",
			"warn",
			"--collision.mingap-factor",
			"2",
			"--random",
			"true",
			"--no-warnings",
		]

		traci.start(sumoCmd)

		self.egoCarID = "ego"

		self.state = self.reset()

	def adversaries(self):

		for i in range(10):
			try:
				if str(i) in traci.vehicle.getIDList():
					traci.vehicle.remove(str(i))
			except:
				pass
			id_car = str(i)
			offset = random.randint(10, 80)
			traci.vehicle.addFull(id_car, 'routeEgo', depart=None, departPos=str(i*90+offset), departSpeed='0', departLane='random', typeID='vType1')
			traci.vehicle.setSpeedMode(id_car, int('00000',2))
			traci.vehicle.setSpeed(id_car, 3)
			traci.vehicle.setLaneChangeMode(id_car, 0)
			traci.simulationStep()
			i+=1
		#add adversary cars in the opposite line
		for i in range(10, 15):
			try:
				if str(i) in traci.vehicle.getIDList():
					traci.vehicle.remove(str(i))
			except:
				pass
			id_car = str(i)
			offset = random.randint(10, 80)
			traci.vehicle.addFull(id_car, 'adversary', depart=None, departPos=str((i-10)*95+offset), departSpeed='0', departLane='random', typeID='vType1')
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

	def _reward_right_lante(self, amount):
		if self.egoCarID in traci.vehicle.getIDList():
			if traci.vehicle.getLaneID(self.egoCarID) == "E0_0":
				return amount
		return 0

	def _reward_overtaking(self, amount):
		adelantamiento = 0
		if self.inside_cars != []:
			inside_cars, _ = self._get_vehicles_in_front()
			if inside_cars != []:
				if inside_cars[0][0] != self.inside_cars[0][0] and self.inside_cars[0][0] in traci.vehicle.getIDList():
					adelantamiento = amount
			else:
				adelantamiento = amount
			self.inside_cars = inside_cars
		else:
			self.inside_cars, _ = self._get_vehicles_in_front()
		return adelantamiento

	#if egocar is in the exterior line and there are no cars in the interior line
	def _reward_wrong_overtake(self, prev_vel):
		if self.egoCarID in traci.vehicle.getIDList():
			inside_cars, _ = self._get_vehicles_in_front()
			if traci.vehicle.getLaneID(self.egoCarID) != "E0_0": # si estoy en el carril exterior
				# si no hay coches en el interior
				if inside_cars == []:
					return 0
				else:
					# si hay coches en el interior pero estan muy lejos, no le doy premio por estar a la  izquierda
					x_ego, _ = self._coords_ego()
					dist = 1 - abs((inside_cars[0][1] - x_ego) / self.max_range)
					if dist < 0.9:
						return 0
			else: # si estoy en el carril interior
				if inside_cars != []:
					x_ego, _ = self._coords_ego()
					dist = 1 - abs((inside_cars[0][1] - x_ego) / self.max_range)
					if round(dist, 1) >= 0.9: # si hay coches en el interior y estan muy cerca, le quito premio 
						return 0
		return prev_vel

	def _reward(self):
		reward = 0
		done = False
		timeout = 220
		overtake_complete = 0

		time = traci.simulation.getTime() - self.time_reset

		if  time > timeout:
			self.timeout = 1
			reward = -9000
			done = True
		x_ego, _ = self._coords_ego()
		if  x_ego < 2:
			self.success = 1
			done = True
		if self.collision:
			reward  = -9000
			done = True
			
		
		if not done:
			is_inside = self._reward_right_lante(1)
			v  = self._reward_wrong_overtake(traci.vehicle.getSpeed(self.egoCarID))
			overtake_complete = self._reward_overtaking(20)
			if overtake_complete != 0 and v == 0:
				v = traci.vehicle.getSpeed(self.egoCarID)
			reward = round(v * (is_inside + overtake_complete), 2)
			'''if v < 4.5:
				reward = 0'''
			
		self.total_reward += reward
		if done:
			self.time_reset = traci.simulation.getTime()

		return done, reward

	def _addEgoCar(self):
		for i in range(np.random.randint(40,50)):
			traci.simulationStep()
		
		#Start the ego vehicle
		if self.egoCarID not in traci.vehicle.getIDList():
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
				lane_adv = traci.vehicle.getLaneID(id)
				v_adv = round(traci.vehicle.getSpeed(id), 1)
				vehicles_in_range.append([id, x_adv, lane_adv, v_adv])
		return vehicles_in_range

	def _get_vehicles_in_front(self):
		adver_in_range = self._get_vehicles_in_range()
		adver_same_line = []
		adver_opposite_line = []
		x_ego, _ = self._coords_ego()
		for i in range(len(adver_in_range)):
			x_adv = adver_in_range[i][1] - 10#8
			lane_adv = adver_in_range[i][2]
			if x_adv < x_ego: # in front
				if lane_adv == "E0_0":
					adver_same_line.append(adver_in_range[i])
				else:
					adver_opposite_line.append(adver_in_range[i])

		#Order by distance, the first one is the farthest
		adver_same_line.sort(key=lambda x: -x[1])
		adver_opposite_line.sort(key=lambda x: -x[1])

		return adver_same_line, adver_opposite_line

	def _observation(self):
		d_if=d_is=d_of=d_os= 0
		v_if=v_of=v_is=v_os= 0
		v_ego = 1
		lane = -1
		v_max = 6
		self.state["adversaries"] = np.array([1] * self.obs_space_adver)
		self.state["ego"] = np.array([self.obs_space_ego])

		if self.egoCarID in traci.vehicle.getIDList():
			adver_same_line, adver_opposite_line = self._get_vehicles_in_front()
			x_ego, _ = self._coords_ego()		

			if adver_same_line != []:
				d_if = 1 - abs((adver_same_line[0][1] - x_ego) / self.max_range)
				v_if = adver_same_line[0][3] / v_max
				if len(adver_same_line) > 1:
					d_is = 1 - abs((adver_same_line[1][1] - x_ego) / self.max_range)
					v_is = adver_same_line[1][3] / v_max
			if adver_opposite_line != []:
				d_of =  1 - abs((adver_opposite_line[0][1] - x_ego) / self.max_range)
				v_of = adver_opposite_line[0][3] / v_max
				if len(adver_opposite_line) > 1:
					d_os = 1 - abs((adver_opposite_line[1][1] - x_ego) / self.max_range)
					v_os = adver_opposite_line[1][3] / v_max
			self.state["adversaries"] = np.array([d_if, v_if, d_is, v_is , d_of, v_of, d_os, v_os])
			
			for i in range(len(self.state["adversaries"])):
				if self.state["adversaries"][i] != int(self.state["adversaries"][i]) and self.state["adversaries"][i] != 1.0 and self.state["adversaries"][i] != 0.0:
					self.state["adversaries"][i] = round(self.state["adversaries"][i], 1)

			#print(self.state["adversaries"])
			if self.egoCarID in traci.vehicle.getIDList():
				lane = 0
				if traci.vehicle.getLaneID(self.egoCarID) == "E0_0":
					lane = 1
			v_ego = round(traci.vehicle.getSpeed(self.egoCarID) / v_max, 2)
			self.state["ego"] = np.array([lane, v_ego])

		return self.state

	def _action(self, action):
		if action == 0 :#right
			traci.vehicle.changeLaneRelative(self.egoCarID, -1, 0)
		elif action == 1:
			traci.vehicle.changeLaneRelative(self.egoCarID, 0, 0)
		elif action == 2:#left
			traci.vehicle.changeLaneRelative(self.egoCarID, 1, 0)
		pass
		
	def _collision(self):
		if self.egoCarID in traci.simulation.getCollidingVehiclesIDList():
			self.collision = 1
		pass




