# Import GYM 
from gym import Env
from gym.spaces import Discrete, Box, Dict

# Import helpers
import numpy as np
import random
import math
import time
import os
from random import choice

# Carla
import carla

#ROS
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from t4ac_msgs.msg import CarControl


class CustomEnv(Env):
    def __init__(self, render=False):
        self.N_actions = 2
        self.action_space = Discrete(self.N_actions)
        self.observation_space = Dict({'ego':Box(0,1,shape=(2,)), 'adversaries':Box(0,1,shape=(8,)),})
        self.state = self.observation_space.sample()

        # Carla Config
        os.system('cd ~/carla/PythonAPI/util/ && python3 config.py -m Town03')
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        world_transform = carla.Transform(carla.Location(x=84, y=-115, z=150), carla.Rotation(yaw=180, pitch = -90))
        self.world.get_spectator().set_transform(world_transform)
        self.map = self.world.get_map()
        blueprint_library = self.world.get_blueprint_library()
        self.adversary_bp = blueprint_library.find('vehicle.tesla.model3')

        weather = carla.WeatherParameters(sun_altitude_angle=90.0)

        self.world.set_weather(weather)

        if render == False:
            settings = self.world.get_settings()
            self.world.apply_settings(settings)
            settings.no_rendering_mode = True
            self.world.apply_settings(settings)

        actors = self.world.get_actors().filter('vehicle.*.*')
        for _, actor in enumerate(actors):
                actor.destroy()

        # Ego vehicle
        ego_vehicle_bp = random.choice(blueprint_library.filter('vehicle.audi.a2'))
        ego_vehicle_transform = carla.Transform(carla.Location(x=84, y=-85, z=10), carla.Rotation(yaw=270))
        self.ego_vehicle = self.world.spawn_actor(ego_vehicle_bp,ego_vehicle_transform)
        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
        self.collision_sensor = self.world.spawn_actor(blueprint_library.find('sensor.other.collision'),
                                        carla.Transform(), attach_to=self.ego_vehicle)
        self.collision_sensor.listen(lambda event: self.function_handler(event))

        # Control PI
        self.init = False
        self.Kp = 0.15
        self.Ki = 0.002
        self.actual_speed = 0
        self.errorSum = 0
        self.action = 1
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
        os.system('roslaunch t4ac_global_planner_ros global_planner.launch map_name:=Town03 &') 
        os.system('roslaunch t4ac_map_monitor_ros map_visualizator_aux.launch map_name:=Town03 &')         
        os.system('roslaunch t4ac_lqr_ros t4ac_lqr_ros.launch &')
        time.sleep(10)
        self.localization_pub()
        os.system('./start.sh')
    
        self.state = self.reset()

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

        if self.action == 0 and self.init:
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=-cmd_vel.steer, brake=brake))
            # self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0.6, steer=0, brake=0))
            self.force_action = True
        elif self.action == 1 and self.init:
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))

        self.throttle = throttle
        self.steer = -cmd_vel.steer
        self.brake = brake

    def function_handler(self, event):
            self.collision = True

    def step(self,action):

        self.world.tick()
        self.localization_pub()

        self.action = action

        self.state = self._obs()

        done, reward = self._reward(action)

        if self.force_action:
            while not done:
                self.world.tick()
                self.localization_pub()
                self.action = action
                self.state = self._obs()
                done, reward = self._reward(action)

        info = {'time' : time.time() - self.time_reset}

        time.sleep(0.05)

        return self.state, reward, done, info

    def reset(self):
        
        # Actions
        self.init = False
        self.force_action = False
        self.action = 1

        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)

        tm = self.client.get_trafficmanager(8000)
        tm_port = tm.get_port()
        tm.global_percentage_speed_difference(10)

        actors = self.world.get_actors().filter('vehicle.*.*')

        for _, actor in enumerate(actors):
            if(actor.id != self.ego_vehicle.id):
                actor.destroy()

        # Adversaries
        routes = ["Right", "Straight", "Left"]
        route = [choice(routes)]
        y = -134
        x = -5
        n_vehicles = 2
        for _ in range(n_vehicles):
            x = x + 30
            adversary_transform = carla.Transform(carla.Location(x=x, y=y, z=8), carla.Rotation(yaw=0))
            actor = self.world.try_spawn_actor(self.adversary_bp,adversary_transform)
            time.sleep(0.1)
            actor.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
            actor.set_autopilot(True,tm_port)
            tm.ignore_lights_percentage(actor,100)
            tm.distance_to_leading_vehicle(actor,10)
            tm.set_route(actor, route)
        y = -135.5
        x = 190
        for _ in range(n_vehicles):
            x = x - 30
            adversary_transform = carla.Transform(carla.Location(x=x, y=y, z=10), carla.Rotation(yaw=180))
            actor = self.world.try_spawn_actor(self.adversary_bp,adversary_transform)
            time.sleep(0.1)
            actor.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
            actor.set_autopilot(True,tm_port)
            tm.ignore_lights_percentage(actor,100)
            tm.distance_to_leading_vehicle(actor,10)
            tm.set_route(actor, route)

        self.ego_vehicle.set_transform(carla.Transform(carla.Location(x=84, y=-85, z=8), carla.Rotation(yaw=270)))

        # Reward parametres init
        self.collision = 0
        self.success = 0
        self.timeout = 0

        time.sleep(1)
        self.time_reset = time.time()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        self.world.apply_settings(settings)

        self._reach_intersection()

        self.init = True

        self.state = self._obs()

        return self.state
      
    def _reward(self, action):

        done = False
        timeout = 20
        time_diff = time.time() - self.time_reset
        rt = 0
        v = math.sqrt(pow(self.ego_vehicle.get_velocity().x,2) + pow(self.ego_vehicle.get_velocity().y,2))

        if  time_diff > timeout:
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
                 0.001
                 

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
                if adversaries_yaw[i] < -80 and adversaries_yaw[i] > -100:
                    d_down.append(round(-adversaries_y[i],2) / d_max)
                    v_down.append(adversaries_v[i] / v_max) 
                elif adversaries_yaw[i] > 80 and adversaries_yaw[i] < 100:
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

    def _reach_intersection(self):
        
        while(self.ego_vehicle.get_transform().location.y + 134 > 20):
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=self.throttle, steer=self.steer, brake=self.brake))
            self.world.tick()
            self.localization_pub()
            time.sleep(0.05)
        
        i = 0
        for i in range(20):
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
            self.world.tick()
            self.localization_pub()
            i += 1
            time.sleep(0.05)
    

    