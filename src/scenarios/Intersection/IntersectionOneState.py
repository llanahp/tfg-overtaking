# Import GYM 
from gym import Env
from gym.spaces import Discrete, Box, Dict

# Import helpers
import numpy as np
import math
import time

# Carla
import carla

class CustomEnv(Env):
    def __init__(self, render=False):
        self.N_actions = 2
        self.action_space = Discrete(self.N_actions)
        self.observation_space = Dict({'ego':Box(0,1,shape=(2,)), 'adversaries':Box(0,1,shape=(8,)),})
        self.state = self.observation_space.sample()

        # Carla Config
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()

        weather = carla.WeatherParameters(sun_altitude_angle=90.0, cloudiness=80)
        self.world.set_weather(weather)

        # Ego vehicle
        actors = self.world.get_actors().filter('vehicle.*.*')
        for actor in actors:
            if actor.type_id == 'vehicle.audi.a2':
                self.ego_vehicle = actor
                self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
                break

        # Observation matrix
        self.range = 30

        blueprint_library = self.world.get_blueprint_library()
        self.adversary_bp = blueprint_library.find('vehicle.tesla.model3')

    def step(self, action):

        self.state = self._obs()

        done, reward = self._reward(action)

        info = {}

        return self.state, reward, done, info

    def reset(self):
      
        self.state = self._obs()

        return self.state
      
    def _reward(self, action):

        reward = 1
        done = False          

        return done, reward

    def _obs(self):

        self.state["adversaries"] = np.array([1] * 8)
        self.state["ego"] = np.array([1] * 2)

        d_max = 100
        v_max = 5
        adversaries_yaw = []
        adversaries_y = []
        adversaries_v = []
        d_up = []
        v_up = []
        d_down = []
        v_down = []
        v_ego = 1
        d_ego = 1

        vehicles = self.world.get_actors().filter('vehicle.*.*')
        for vehicle in vehicles:
            if vehicle.id != self.ego_vehicle.id and self.vehicle_in_range(vehicle):
                x_adv, y_adv = self.local_to_global(vehicle)
                adversaries_y.append(y_adv)
                adversaries_v.append(math.sqrt(pow(vehicle.get_velocity().x,2) + pow(vehicle.get_velocity().y,2)))  
                adversaries_yaw.append(self.ego_vehicle.get_transform().rotation.yaw - vehicle.get_transform().rotation.yaw)

        if adversaries_yaw:
            for i in range(len(adversaries_yaw)):
                if adversaries_yaw[i] < -65 and adversaries_yaw[i] > -115:
                    d_down.append(round(-adversaries_y[i],2) / d_max)
                    v_down.append(adversaries_v[i] / v_max) 
                elif adversaries_yaw[i] > 65 and adversaries_yaw[i] < 115:
                    d_up.append(round(adversaries_y[i],2) / d_max)
                    v_up.append(adversaries_v[i] / v_max)     
            
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
        self.state["ego"] = np.array([v_ego, d_ego])  

        return self.state

    def _acc(self):
        velocity = 8
        vehicles = self.world.get_actors().filter('vehicle.*.*')
        for vehicle in vehicles:
            if vehicle.type_id != 'vehicle.audi.a2' and self.vehicle_in_range(vehicle):
                x_adv, y_adv = self.local_to_global(vehicle)

                if y_adv < 1 and y_adv > -1:
                    velocity = 8 * x_adv / self.range
                if velocity < 1.5:
                    velocity = 0
        
        return velocity

    def vehicle_in_range(self, adversary):
        if math.sqrt(pow(self.ego_vehicle.get_transform().location.x - adversary.get_transform().location.x ,2) + 
                     pow(self.ego_vehicle.get_transform().location.y - adversary.get_transform().location.y ,2)) < self.range:
                     return True; return False 

    def local_to_global(self, adversary):
        x_ego = self.ego_vehicle.get_transform().location.x
        y_ego = self.ego_vehicle.get_transform().location.y
        yaw = (self.ego_vehicle.get_transform().rotation.yaw * math.pi) / 180
        x_adv = adversary.get_transform().location.x
        y_adv = adversary.get_transform().location.y
        x = (x_adv - x_ego)*math.cos(yaw) + (y_adv - y_ego)*math.sin(yaw)
        y = -(x_adv - x_ego)*math.sin(yaw) + (y_adv - y_ego)*math.cos(yaw)
        return x, y




        


    

    