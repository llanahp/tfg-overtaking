# Import GYM 
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete

# Import helpers
import os, sys
import numpy as np
from random import randint, choice
import copy
import time
import math 

# Imports Prediction Modules
#from scenarios.SMARTS.prediction.predictions_utils import Motion_Predictor

# Import CARLA
import carla

#ROS
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from t4ac_msgs.msg import CarControl

class CustomEnv(Env):
    def __init__(self, render=True):
        self.N_actions = 2
        self.action_space = Discrete(self.N_actions)
        self.n_features = 2
        self.n_vehicles =  11
        self.n_states = 4
        self.n_obs = 50

        self.observation_space = Dict({'ego':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle A':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle B':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle C':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle D':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle E':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle F':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle G':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle H':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle I':Box(0,1,shape=(self.n_states, self.n_features)), \
                                       'vehicle J':Box(0,1,shape=(self.n_states, self.n_features))
                                       })

        self.list_of_keys = [key for key in self.observation_space.keys()]
        self.empty = self.observation_space.sample()
        self.list_of_ids = {}


        # Define number of vehicles
        for num_adv in range(self.n_vehicles):
            current_adv = self.list_of_keys[num_adv]
            self.empty[current_adv] = np.zeros((self.n_states, self.n_features))
        #     self.empty[current_adv][:,0] = -1
        #     self.empty[current_adv][:,3] = 1

        self.timestep = 0.1
        self.timestamp = 0
        self.render = render
        self._run_carla()

        self.egoCarID = "ego"
        self.adversaries_keys = dict()
        self.adversaries_id_cnt = 1

        #self.write_csv = False
        #self.predict = True

    # This function is called every time we take a step in the environment
    # It sends the action to the CARLA simulator and returns the new state
    def localization_pub(self):
        localization_msg = Odometry()
        localization_msg.header.frame_id="map"
        localization_msg.header.stamp = rospy.Time.now()
        localization_msg.child_frame_id = "ego_vehicle" 
        transform = self.ego_vehicle.get_transform()
        localization_msg.pose.pose.position.x = transform.location.x
        localization_msg.pose.pose.position.y = -transform.location.y
        localization_msg.pose.pose.position.z = transform.location.z
        localization_msg.pose.pose.orientation = self.euler_to_quaternion(transform.rotation)
        self.pub_localization.publish(localization_msg)

    def euler_to_quaternion(self, rotation):

        roll = math.radians(rotation.roll) 
        pitch = math.radians(rotation.pitch)
        yaw = - math.radians(rotation.yaw)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        ros_quaternion = Quaternion(w=qw, x=qx, y=qy, z=qz)

        return ros_quaternion


    #TODO Intentar quitar esto y que lo controle el simulador (Carla)
    # This function is used to control the ego vehicle (throttle, steer, brake)
    def control_cb(self, cmd_vel):
        self.actual_speed = math.sqrt(pow(self.ego_vehicle.get_velocity().x,2)+pow(self.ego_vehicle.get_velocity().y,2))
        errorSpeed = cmd_vel.velocity - self.actual_speed #distance away from setpoint
        self.errorSum += (errorSpeed * self.Ki)
        if (self.errorSum > 0.5):
            self.errorSum = 0.5
        if (self.errorSum < -0.5):
            self.errorSum = -0.5

        throttle = ((errorSpeed * self.Kp) + self.errorSum)
        
        brake = 0.0
        if (cmd_vel.velocity == 0):
            self.errorSum = 0 #Reset PI

        if (throttle < 0):
            brake = -throttle 
            throttle = 0 
        if (throttle > 1):
            throttle = 1

        # Apply control
        if self.action == 1 and self.init:
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=-cmd_vel.steer, brake=brake))
            # self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0.6, steer=0, brake=0))
            self.force_action = True
        # Apply brake
        elif self.action == 0 and self.init:
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))

        self.throttle = throttle
        self.steer = -cmd_vel.steer
        self.brake = brake

    def function_handler(self, event):
            self.collision = True

    def step(self, action):

        self.timestamp += 1

        self.world.tick()
        self.localization_pub()

        self.action = action
        state = self._obs()
        done, reward = self._reward(action)

        if self.force_action:
            while not done:
                self.action = 1
                self.world.tick()
                self.localization_pub()
                done, reward = self._reward(action)

        info = {'time' : time.time() - self.time_reset, 'actions' : self.action}
        time.sleep(0.05)

        return state, reward, done, info

    def reset(self):
        # Actions
        self.init = False
        self.force_action = False
        self.action = 0

        settings = self.world.get_settings()
        #TODO Antes estaba solo la siguiente linea 
        #settings.synchronous_mode = False

        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # Tiempo entre cada paso de la simulación en segundos


        self.world.apply_settings(settings)

        tm = self.client.get_trafficmanager(8000)
        tm_port = tm.get_port()
        tm.global_percentage_speed_difference(10)

        actors = self.world.get_actors().filter('vehicle.*.*')

        # Destroy all actors except the ego vehicle
        for _, actor in enumerate(actors):
            if(actor.id != self.ego_vehicle.id):
                actor.destroy()

        # Adversaries random spawn and routes
        routes = ["Right", "Straight", "Left"]
        route = [choice(routes)]
        n_vehicles = 2
        # From one side
        y = -134
        x = 20
        for _ in range(n_vehicles):
            x = x + 15
            adversary_transform = carla.Transform(carla.Location(x=x, y=y, z=8), carla.Rotation(yaw=0))
            actor = self.world.try_spawn_actor(self.adversary_bp,adversary_transform)
            time.sleep(0.5)
            if actor:
                actor.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
                actor.set_autopilot(True,tm_port)
                tm.ignore_lights_percentage(actor,100)
                tm.distance_to_leading_vehicle(actor,10)
                tm.set_route(actor, route)
        
        # From one side
        y = -135.5
        x = 130
        for _ in range(n_vehicles):
            x = x - 15
            adversary_transform = carla.Transform(carla.Location(x=x, y=y, z=10), carla.Rotation(yaw=180))
            actor = self.world.try_spawn_actor(self.adversary_bp,adversary_transform)
            time.sleep(0.5)
            if actor:
                actor.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
                actor.set_autopilot(True,tm_port)
                tm.ignore_lights_percentage(actor,100)
                tm.distance_to_leading_vehicle(actor,10)
                tm.set_route(actor, route)

        # Ego vehicle spawn
        self.ego_vehicle.set_transform(carla.Transform(carla.Location(x=84, y=-85, z=8), carla.Rotation(yaw=270)))

        # Reward parametres init
        self.collision = 0
        self.success = 0
        self.timeout = 0
        self.total_reward = 0

        time.sleep(1)
        self.time_reset = time.time()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        self.world.apply_settings(settings)

        self.list_of_ids.clear()
        self.adversaries_keys.clear()
        self.adversaries_id_cnt = 1

        #self._reach_intersection()

        self.init = True
        state = self._obs()
        return state

    def _reward(self, action):
        done = False
        # Elapsed time
        time_elapse = time.time() - self.time_reset

        timeout = 30 #20

        if  time_elapse > timeout:
            self.timeout = 1
            done = True
            print("-------- timeout! --------")
            print(self.ego_vehicle.get_transform().location.x,self.ego_vehicle.get_transform().location.y)

        if self.ego_vehicle.get_transform().location.y < -150:
            self.success = 1
            done = True
            print("-------- success! --------")
            print(self.ego_vehicle.get_transform().location.x,self.ego_vehicle.get_transform().location.y)

        if self.collision:
            done = True
            print("-------- collision! --------")
            print(self.ego_vehicle.get_transform().location.x,self.ego_vehicle.get_transform().location.y)
        
        # TODO: reward function
        '''

        rt = 0
        v = math.sqrt(pow(self.ego_vehicle.get_velocity().x,2) + pow(self.ego_vehicle.get_velocity().y,2))


        ks = 1
        kc = -2
        kt = -0.2
        kv = 0.0002
        if done:
            rt = (kt * time_diff) / timeout
        reward = ks * self.success + \
                 kc * self.collision + \
                 kv * v + \
                 rt - \
                 0.001'''
        reward = 0      

        return done, reward
    
    def _obs(self):
        state = copy.deepcopy(self.empty)
        obs = {}

        adversaries = []
        '''
        vehicles = self.world.get_actors().filter('vehicle.*.*')
        for vehicle in vehicles:
            if vehicle.id != self.ego_vehicle.id and self.vehicle_in_range(vehicle):
                if not vehicle.id in self.adversaries_keys.keys():
                    self.adversaries_keys[vehicle.id] = self.adversaries_id_cnt
                    self.adversaries_id_cnt += 1

                adversaries.append([self.adversaries_keys[vehicle.id], vehicle.get_transform().location.x, vehicle.get_transform().location.y])

        if "ego" in self.list_of_ids:
            self.list_of_ids["ego"].append([self.ego_vehicle.get_transform().location.x, self.ego_vehicle.get_transform().location.y, 1])
        else:
            self.list_of_ids["ego"] = [[0, 0, 0] for _ in range(self.n_obs)]
            self.list_of_ids["ego"].append([self.ego_vehicle.get_transform().location.x, self.ego_vehicle.get_transform().location.y, 1])

        for adv in range(len(adversaries)):
            adv_id = adversaries[adv][0]
            x_adv = adversaries[adv][1]
            y_adv = adversaries[adv][2]

            if adv_id in self.list_of_ids:
                self.list_of_ids[adv_id].append([x_adv, y_adv, 1])
            else:
                self.list_of_ids[adv_id] = [[0, 0, 0] for _ in range(self.n_obs)]
                self.list_of_ids[adv_id].append([x_adv, y_adv, 1])

        for adv in range(len(adversaries)+1):
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
                    current_adv = self.list_of_keys[num_adv]
                    if num_adv < len(predictions)-1:
                        state[current_adv] = predictions[num_adv]
                    else:
                        break
            else:
                for num_adv in range(self.n_vehicles):
                    current_adv = self.list_of_keys[num_adv]
                    if num_adv < len(obs)-1:
                        state[current_adv][0] = [obs[num_adv][49][0], obs[num_adv][49][1]]
                    else:
                        break
        '''
        return state

    '''def _reach_intersection(self):
        
        while(self.ego_vehicle.get_transform().location.y + 134 > 20):
            state = self._obs()
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=self.throttle, steer=self.steer, brake=self.brake))
            self.world.tick()
            self.localization_pub()
            time.sleep(0.05)
        
        i = 0
        for i in range(20):
            state = self._obs()
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
            self.world.tick()
            self.localization_pub()
            i += 1
            time.sleep(0.05)'''



    def vehicle_in_range(self, adversary):
        if math.sqrt(pow(self.ego_vehicle.get_transform().location.x - adversary.get_transform().location.x ,2) + 
                     pow(self.ego_vehicle.get_transform().location.y - adversary.get_transform().location.y ,2)) < self.range:
                     return True; return False 


    def _run_carla(self):
        # Carla Config
        os.system('cd ~/carla/PythonAPI/util/ && python3 config.py -m Town03')
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(2.0) # Timeout for network operations
        self.world = self.client.get_world()

        world_transform = carla.Transform(carla.Location(x=84, y=-115, z=150), carla.Rotation(yaw=180, pitch = -90))
        self.world.get_spectator().set_transform(world_transform)
        self.map = self.world.get_map()
        blueprint_library = self.world.get_blueprint_library()
        self.adversary_bp = blueprint_library.find('vehicle.tesla.model3')


        # Remove buildings
        settings = self.world.get_settings()
        settings.world.unload_map_layer(carla.MapLayer.Props) 
        self.world.apply_settings(settings)



        # Set Weather
        weather = carla.WeatherParameters(sun_altitude_angle=90.0)
        self.world.set_weather(weather)

        if self.render == False:
            settings = self.world.get_settings()
            self.world.apply_settings(settings)
            settings.no_rendering_mode = True
            self.world.apply_settings(settings)
            

        # Destroy all actors 
        actors = self.world.get_actors().filter('vehicle.*.*')
        for _, actor in enumerate(actors):
                actor.destroy()

        # Ego vehicle
        #------------------------------
        ego_vehicle_bp = choice(blueprint_library.filter('vehicle.audi.a2'))
        #Spawn point ego vehicle
        ego_vehicle_transform = carla.Transform(carla.Location(x=84, y=-85, z=10), carla.Rotation(yaw=270))
        self.ego_vehicle = self.world.spawn_actor(ego_vehicle_bp,ego_vehicle_transform)
        # Start ego vehicle
        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
        # Attach collision sensor to ego vehicle
        self.collision_sensor = self.world.spawn_actor(blueprint_library.find('sensor.other.collision'),
                                        carla.Transform(), attach_to=self.ego_vehicle)
        # Attach callback to collision sensor
        self.collision_sensor.listen(lambda event: self.function_handler(event))
        #------------------------------
        
        # Control PI, (throttle, steer, brake)
        self.init = False
        self.Kp = 0.15
        self.Ki = 0.002
        self.actual_speed = 0
        self.errorSum = 0
        self.action = 0
        self.throttle = 0
        self.steer = 0
        self.brake = 0

        # Observation matrix
        self.range = 50

        # ROS init
        os.system('roscore &')
        rospy.init_node('env_node', anonymous=True)
        rospy.Subscriber("/t4ac/control/cmd_vel", CarControl, self.control_cb)
        self.pub_localization = rospy.Publisher('/t4ac/localization/pose',Odometry,queue_size=1)
        os.system('roslaunch t4ac_global_planner_ros planning.launch map_name:=Town03 &') 
        os.system('roslaunch t4ac_map_monitor_ros mapping.launch map_name:=Town03 &')         
        os.system('roslaunch t4ac_lqr_ros t4ac_lqr_ros.launch &')
        #os.system('rviz &')  # Start rviz
        time.sleep(10)
        self.localization_pub()
        os.system('./start.sh')


        