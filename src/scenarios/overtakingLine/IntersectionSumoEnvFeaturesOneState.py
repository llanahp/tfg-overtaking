# Import GYM 
from operator import ne
from turtle import end_fill, st
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete
import uuid
from colorama import Fore, Back, Style, init
from math import log


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
		self.action_space = Discrete(3) #Box(low=0, high=1, shape=(2,))# Number of actions
		self.n_adversaries = 4  # 2 going up and 2 going down
		self.n_features = 2 # distance to type_vehicle, speed
		self.n_features_ego = 2 # line and speed
		self.line_position = 2
		self.observation_space = Dict({
						'ego':Box(0,1,shape=(self.n_features_ego,)), \
						'adversaries':Box(0,1,shape=(self.n_adversaries*self.n_features,)), \
						})
		self.state = self.observation_space.sample()
		self.timestep = 0.4
		self.prev_action = -1
		self.spawn_cars = 0
		self.inside_cars = []
		sumo = "sumo-gui" if render else "sumo"

		sumoCmd = [
			sumo,
			"-c",
			"scenarios/overtakingLine/sumo_config/overtaking.sumocfg",
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



	# Return the coordinates of the ego car
	def _get_coords_ego(self):
		x_ego = round(traci.vehicle.getPosition(self.egoCarID)[0],2)
		y_ego = round(traci.vehicle.getPosition(self.egoCarID)[1],2)
		return x_ego, y_ego

	# Round the distance to other vehicles
	def _round_distance(self, dist):
		if dist == 0.0 or dist == 1.0:
			return int(dist)
		return dist

	def _get_current_lane_ego(self):
		#existe ego car y  carril exterior
		if self.egoCarID in traci.vehicle.getIDList() and traci.vehicle.getLaneID(self.egoCarID)[0] == "-":
			return 1
		return 0

	# saber el vehiculo que estan por delante de mi y a la distancia que esta
	def _get_vehicle_intirior(self):
		d_max = 10
		v_max = 8.5

		adversaries_yaw = []
		adversaries_y = []
		adversaries_x = []
		adversaries_v = []
		v_ego = 1
		d_ego = 1

		resul = []


		for id in traci.vehicle.getIDList():
			if id != self.egoCarID and self.vehicle_in_range(id):
				x_adv, y_adv = traci.vehicle.getPosition(id)
				adversaries_y.append(x_adv)
				adversaries_x.append(y_adv)
				adversaries_v.append(traci.vehicle.getSpeed(id))
				adversaries_yaw.append(id)
		y_ego, x_ego = self._get_coords_ego()

		if adversaries_yaw:
			for i in range(len(adversaries_yaw)):
				id = adversaries_yaw[i]
				x =  round(adversaries_x[i], 2)
				y =  round(adversaries_y[i], 2)
				v =  round(adversaries_v[i], 2)
				if abs(x - x_ego) <= d_max:
					d = abs(y - y_ego)
				elif abs(y - y_ego) <= d_max:
					d = abs(x - x_ego)
				else:
					d = d_max
				d = d / d_max
				d = d / d_max

				d_ego = self.collision_point(d_ego, adversaries_x[i], d_max)
				angle = int(round(traci.vehicle.getAngle(self.egoCarID), 0))
				if abs(x - x_ego) <= d_max:
					if angle >= 265 and angle <= 275 and y - 6 > y_ego: # abajo y hacia la izquierda
						continue
					if angle >= 85 and angle <= 95 and y + 6 < y_ego: # arriba y hacia la derecha
						continue
				elif abs(y - y_ego) <= d_max:
					if angle >= 175 and angle <= 185 and x - 6 > x_ego: # izquierda y hacia abajo
						continue
					if angle >= 355 and angle <= 5 and x + 6 < x_ego:
						continue
				if id.find("inside") != -1 and d != 1.0: #inside line
					resul.append([id, d])
		#ordeno los coches de mayor a menor distancia
		for i in range(len(resul)):
			for j in range(i, len(resul)):
				if resul[i][1] > resul[j][1]:
					aux = resul[i]
					resul[i] = resul[j]
					resul[j] = aux
		if len(resul) >=1:
			return resul[0]
		return []

	def step(self, action):
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
			if (done == False):
				done = self._reach_end_line()
		
		if done:
			print(f"total reward: {self.total_reward}")
			self.time_reset = traci.simulation.getTime()
			self.reset()
			
		traci.simulationStep()
		info = {'time' : traci.simulation.getTime() - self.time_reset}

		return self.state, reward, done, info

	def reset(self):
		for id in traci.vehicle.getIDList():
			traci.vehicle.remove(id)
		traci.simulationStep()
		
		self.prev_action = -1
		self.spawn_cars = 0
		self.swap = not self.swap
		self.spawn_time = traci.simulation.getTime()
		self.time_car_added = traci.simulation.getTime()
		self.inside_cars = []

		self._addCar()

		self.egoCar = False
		self._addEgoCar()
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
		timeout = 250
		reward = 0.0
		time = traci.simulation.getTime() - self.time_reset
		if  time > timeout:
			self.timeout = 1
			done = True
			reward += -4000
		if self.collision:
			done = True
			reward += -4000
		

		if (done == False): 
			is_inside = self._reward_right_lante(1)
			
			# si estoy en el carril exterior y no hay coches en el interior
			v  = self._reward_wrong_action(traci.vehicle.getSpeed(self.egoCarID))
			
			# si se ha producido un adelantamiento
			adelantamiento = self._reward_overtaking(20)
			if adelantamiento != 0 and v == 0:
				v = traci.vehicle.getSpeed(self.egoCarID)
			
			#redondo a 2 decimales
			reward = round(v * (is_inside + adelantamiento), 2)

		
		self.prev_action = action
		self.total_reward += reward
		if done:
			self.time_reset = traci.simulation.getTime()
		return done, reward

	def _collision(self):
		if self.egoCarID in traci.simulation.getCollidingVehiclesIDList():
			self.collision = True

	def _action(self, action):
		if action == 0:
			traci.vehicle.changeLaneRelative(self.egoCarID, -1, 0) #derecha
			
		elif action == 1:
			traci.vehicle.changeLaneRelative(self.egoCarID, 0, 0) #mantenerse en el carril
			
		elif action == 2:
			traci.vehicle.changeLaneRelative(self.egoCarID, 1, 0) #izquierda
			
		pass
	
	def _reach_end_line(self):
		y_ego , x_ego = self._get_coords_ego()
		if x_ego <= -100 and y_ego <= -95:
			return True
		return False

	def _obs(self):
		self.state["adversaries"] = np.array([1] * 8)
		self.state["ego"] = np.array([1] * 2)

		d_max = 10
		v_max = 8.5

		d_exterior = []
		v_exterior = []
		d_intirior = []
		v_intirior = []
		v_ego = 1
		d_ego = 1

		adversaries_yaw = []
		adversaries_y = []
		adversaries_x = []
		adversaries_v = []

		for id in traci.vehicle.getIDList():
			if id != self.egoCarID and self.vehicle_in_range(id):
				x_adv, y_adv = traci.vehicle.getPosition(id)
				adversaries_y.append(x_adv)
				adversaries_x.append(y_adv)
				adversaries_v.append(traci.vehicle.getSpeed(id))
				adversaries_yaw.append(id)
		y_ego, x_ego = self._get_coords_ego()

		if adversaries_yaw:
			for i in range(len(adversaries_yaw)):
				id = adversaries_yaw[i]
				x =  round(adversaries_x[i], 2)
				y =  round(adversaries_y[i], 2)
				v =  round(adversaries_v[i], 2)
				if abs(x - x_ego) <= d_max:
					d = abs(y - y_ego)
				elif abs(y - y_ego) <= d_max:
					d = abs(x - x_ego)
				else:
					d = d_max
				d = d / d_max
				d = d / d_max

				d_ego = self.collision_point(d_ego, adversaries_x[i], d_max)
				angle = int(round(traci.vehicle.getAngle(self.egoCarID), 0))
				if abs(x - x_ego) <= d_max: # si el coche está arriva o abajo
					#+-5 angulo de error (270)
					if angle >= 265 and angle <= 275 and y - 6 > y_ego: # abajo y hacia la izquierda
						continue
					if angle >= 85 and angle <= 95 and y + 6 < y_ego: # arriba y hacia la derecha
						continue
				elif abs(y - y_ego) <= d_max: # si el coche está a la derecha o izquierda
					if angle >= 175 and angle <= 185 and x - 6 > x_ego: # izquierda y hacia abajo
						continue
					if angle >= 355 and angle <= 5 and x + 6 < x_ego:
						continue

				if id.find("inside") != -1: #inside line
					d_intirior.append(d)
					v_intirior.append(v / v_max)
				elif id.find("outside") != -1: #outside line
					d_exterior.append(d)
					v_exterior.append(v / v_max)    

		#me copio las d_intirior y d_exterior
		d_intirior_pre = []
		d_exterior_pre = []
		for item in d_intirior:
			d_intirior_pre.append(item)
		for item in d_exterior:
			d_exterior_pre.append(item)


		for _ in range(8):
			d_exterior.append(1)
			d_intirior.append(1)
			v_exterior.append(1)
			v_intirior.append(1)
		
		d_exterior = np.sort(d_exterior)
		d_intirior = np.sort(d_intirior)

		#Get the positive values (closer distances to the type_vehicle)
		d_exterior = d_exterior[d_exterior>0]
		d_intirior = d_intirior[d_intirior>0]

		v_exterior_i = np.array(v_exterior).size - d_exterior.size
		v_intirior_i = np.array(v_intirior).size - d_intirior.size

		#ordeno las d_pre
		while len(d_intirior_pre) < 2:
			d_intirior_pre.append(1)

		while len(d_exterior_pre) < 2:
			d_exterior_pre.append(1)

		d_intirior_pre = np.sort(d_intirior_pre)
		d_exterior_pre = np.sort(d_exterior_pre)

		# dejo solo 3 decimales en d_intirior_pre y d_exterior_pre
		for i in range(len(d_intirior_pre)):
			d_intirior_pre[i] = round(d_intirior_pre[i], 1)
			if d_intirior_pre[i] == 0.0 or d_intirior_pre[i] == 1.0:
				d_intirior_pre[i] = int(d_intirior_pre[i])
		for i in range(len(d_exterior_pre)):
			d_exterior_pre[i] = round(d_exterior_pre[i], 1)
			if d_exterior_pre[i] == 0.0 or d_exterior_pre[i] == 1.0:
				d_exterior_pre[i] = int(d_exterior_pre[i])
		
		#dejo solo 3 decimales en v_intirior y v_exterior
		for i in range(len(v_intirior)):
			v_intirior[i] = round(v_intirior[i], 2)
			if v_intirior[i] == 0.0 or v_intirior[i] == 1.0:
				v_intirior[i] =  int(v_intirior[i])
			#correcion para que si no ve nada, sea 0 en vez de 1
			if v_intirior[i] == 1:
				v_intirior[i] = 0
		for i in range(len(v_exterior)):
			v_exterior[i] = round(v_exterior[i], 2)
			if v_exterior[i] == 0.0 or v_exterior[i] == 1.0:
				v_exterior[i] = int(v_exterior[i])
			#correcion para que si no ve nada, sea 0 en vez de 1
			if v_exterior[i] == 1:
				v_exterior[i] = 0

		d_int_1 = self._round_distance(1 - d_intirior_pre[0])
		d_int_2 = self._round_distance(1 - d_intirior_pre[1])
		d_ext_1 = self._round_distance(1 - d_exterior_pre[0])
		d_ext_2 = self._round_distance(1 - d_exterior_pre[1])

		self.state["adversaries"] = np.array([d_int_1,v_intirior[v_intirior_i],d_int_2,v_intirior[v_intirior_i+1],d_ext_1,v_exterior[v_exterior_i],d_ext_2,v_exterior[v_exterior_i+1]])
		#print("[", d_int_1, ", ", v_intirior[v_intirior_i], ", ", d_int_2, ", ", v_intirior[v_intirior_i+1], ", ", d_ext_1, ", ", v_exterior[v_exterior_i], ", ", d_ext_2, ", ", v_exterior[v_exterior_i+1], " ]")
		
		#direccion ego en el mapa
		if self.egoCarID in traci.vehicle.getIDList():
			#d_ego = round(round(traci.vehicle.getPosition(self.egoCarID)[0],2) / d_max, 3)
			v_ego = round(traci.vehicle.getSpeed(self.egoCarID) / v_max, 2)
		self.state["ego"] = np.array([self._get_current_lane_ego(), v_ego])
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

	def _addEgoCar(self):
		if self.egoCarID not in traci.vehicle.getIDList():
			traci.vehicle.addFull(self.egoCarID, 'routeEgo', depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType0')
			traci.vehicle.setSpeedMode(self.egoCarID, int('111111',2))
			traci.vehicle.setSpeed(self.egoCarID, 6)
			traci.vehicle.setLaneChangeMode(self.egoCarID, 0)

	def _add_adversaries_cars(self, velocity, idd):
		car = ["vType1", "vType2", "vType3", "vType4", "vType5"]
		routeOutsido = ["rt10", "rt11", "rt12", "rt13", "rt14", "rt15", "rt16", "rt17", "rt18"]
		if self.type_vehicle == 0:
			if self.line_position != 2:
				traci.vehicle.addFull(idd, choice(routeOutsido), depart=None, departPos='0', departSpeed='0', departLane='0', typeID=choice(car))
				traci.vehicle.setSpeedMode(idd, int('101111',2))
				traci.vehicle.setSpeed(idd, velocity)
				traci.vehicle.setLaneChangeMode(idd, 0)
		elif self.type_vehicle == 1:
			if self.line_position != 2:
				traci.vehicle.addFull(idd, choice(routeOutsido), depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType2')
				traci.vehicle.setSpeedMode(idd, int('101111',2))
				traci.vehicle.setSpeed(idd, velocity)
				traci.vehicle.setLaneChangeMode(idd, 0)
		elif self.type_vehicle == 2:
			if self.line_position != 2:
				traci.vehicle.addFull(idd, choice(routeOutsido), depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType3')
				traci.vehicle.setSpeedMode(idd, int('101111',2))
				traci.vehicle.setSpeed(idd, velocity)
				traci.vehicle.setLaneChangeMode(idd, 0)

	def _addCar(self):
		max_cars = 7
		car = ["vType1", "vType2", "vType3", "vType4", "vType5"]
		routeInside = ["rt3", "rt4", "rt5", "rt6", "rt7", "rt8", "rt9"]
		
		iterador = 0
		for id in traci.vehicle.getIDList():
			if id == self.egoCarID:
				continue
			traci.vehicle.remove(id)
		traci.simulationStep()
		while True:
			iterador += 1
			if iterador >= max_cars:
				break
			
			if self.spawn_cars >= max_cars or len(traci.vehicle.getIDList()) >= max_cars:
				break 
			self.line_position = randint(1,2)
			if self.line_position == 2:
				idu = str(uuid.uuid4()) + "inside"
			idd = str(uuid.uuid4()) + "outside"
			self.type_vehicle = randint(0,2)
			#add adversary cars
			#self._add_adversaries_cars(2, idd)
			if self.type_vehicle == 0:
				if self.line_position == 2:
					traci.vehicle.addFull(idu, choice(routeInside), depart=None, departPos='0', departSpeed='0', departLane='0', typeID=choice(car))
					traci.vehicle.setSpeedMode(idu, int('101111',2))
					traci.vehicle.setSpeed(idu, 3)
					traci.vehicle.setLaneChangeMode(idu, 0)
			elif self.type_vehicle == 1:
				if self.line_position == 2:
					traci.vehicle.addFull(idu, choice(routeInside), depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType2')
					traci.vehicle.setSpeedMode(idu, int('101111',2))
					traci.vehicle.setSpeed(idu, 3)
					traci.vehicle.setLaneChangeMode(idu, 0)
			elif self.type_vehicle == 2:
				if self.line_position == 2:
					traci.vehicle.addFull(idu, choice(routeInside), depart=None, departPos='0', departSpeed='0', departLane='0', typeID='vType3')
					traci.vehicle.setSpeedMode(idu, int('110111',2))
					traci.vehicle.setSpeed(idu, 3)
					traci.vehicle.setLaneChangeMode(idu, 0)

			self.spawn_cars += 1
			if  self.spawn_cars >= max_cars or len(traci.vehicle.getIDList()) >= max_cars:
				break 
			if iterador >= max_cars:
				break
			traci.simulationStep()
		traci.simulationStep()	
		pass

	def _free_spawn(self):
		if traci.simulation.getTime() - self.spawn_time > 0:
			self.spawn_time = traci.simulation.getTime()
			return True
		else:
			return False

	def _reward_right_lante(self, amount):
		if self.egoCarID in traci.vehicle.getIDList():
			if traci.vehicle.getLaneID(self.egoCarID).startswith("-"):
				return amount
		return 0
	
	def _reward_change_action(self, action, amount):
		if (self.prev_action != action and self.prev_action != -1):
			return amount
		return 0
	
	def _reward_overtaking(self, amount):
		adelantamiento = 0
		if self.inside_cars != []:
			inside_cars = self._get_vehicle_intirior()
			if inside_cars != []:
				if inside_cars[0] != self.inside_cars[0] and self.inside_cars[0] in traci.vehicle.getIDList() and inside_cars[0].find("inside") != -1:
					adelantamiento = amount
			else:
				adelantamiento = amount
			self.inside_cars = inside_cars
		else:
			self.inside_cars = self._get_vehicle_intirior()
		return adelantamiento

	def _reward_wrong_action(self, prev_vel):
		if self.egoCarID in traci.vehicle.getIDList():
			if traci.vehicle.getLaneID(self.egoCarID).startswith("-") == False:
				# si no hay coches en el interior
				inside_cars = self._get_vehicle_intirior()
				if inside_cars == []:
					return 0
				else:
					# si hay coches en el interior pero estan muy lejos, no le doy premio por estar a la  izquierda
					if inside_cars[1] < 0.9:
						return 0
		return prev_vel