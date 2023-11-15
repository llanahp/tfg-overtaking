# Import helpers
import sys
import glob
import os
import numpy as np
import time
import math

import random
from random import randint

#CARLA
import carla

#ROS
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from t4ac_msgs.msg import CarControl
from std_msgs.msg import Bool

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

class AutonomousAgent():
    def __init__(self):

        town = "Town03"

        # Init the scenario in CARLA
        os.system('cd ~/carla/PythonAPI/util/ && python3 config.py -m' + town)
        time.sleep(1.0)
        # os.system('cd ~/carla/PythonAPI/examples/ && python3 no_rendering_mode.py &')
        # time.sleep(1.0)
        # os.system('cd ~/carla/PythonAPI/util/ && python3 config.py --no-rendering')
        # time.sleep(1.0)
        
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        if town == "Town03":
            world_transform = carla.Transform(carla.Location(x=69, y=-7.5, z=150), carla.Rotation(yaw=180, pitch = -90))
        elif town == "Town01":
            world_transform = carla.Transform(carla.Location(x=-13.4, y=-110, z=120), carla.Rotation(yaw=180, pitch = -90))
        elif town == "Town04":
            world_transform = carla.Transform(carla.Location(x=-13.4, y=-110, z=120), carla.Rotation(yaw=180, pitch = -90))

        self.world.get_spectator().set_transform(world_transform)
        self.map = self.world.get_map()
        blueprint_library = self.world.get_blueprint_library()

        weather = carla.WeatherParameters(sun_altitude_angle=90.0)

        self.world.set_weather(weather)

        # Ego vehicle
        actors = self.world.get_actors().filter('vehicle.*.*')
        if actors:
            self.ego_vehicle = actors[0]
        else: 
            ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
            if town == "Town03":
                ego_vehicle_transform = carla.Transform(carla.Location(x=69, y=-7.5, z=10), carla.Rotation(yaw=180))
            elif town == "Town01":
                ego_vehicle_transform = carla.Transform(carla.Location(x=392, y=60, z=0.2), carla.Rotation(yaw=90)) 
            elif town == "Town04":
                ego_vehicle_transform = carla.Transform(carla.Location(x=-13.4, y=-232, z=0.2), carla.Rotation(yaw=90)) 
                adv_transform = carla.Transform(carla.Location(x=-13.4, y=-150, z=0.2), carla.Rotation(yaw=90))
                # Borrar 
                world_transform = carla.Transform(carla.Location(x=-0, y=-183, z=8), carla.Rotation(pitch = -20, yaw=120)) 
                self.world.get_spectator().set_transform(world_transform)
        
            actors = self.world.get_actors().filter('vehicle.*.*')
        
            for _, actor in enumerate(actors):
                    actor.destroy()
            self.ego_vehicle = self.world.spawn_actor(ego_vehicle_bp,ego_vehicle_transform)
            try:
                self.world.spawn_actor(random.choice(blueprint_library.filter('vehicle.audi.tt')),adv_transform)
            except:
                pass

            self.collision_sensor = self.world.spawn_actor(blueprint_library.find('sensor.other.collision'),
                                            carla.Transform(), attach_to=self.ego_vehicle)
            self.collision_sensor.listen(lambda event: self.function_handler(event))

        # Control PI
        self.Kp = 0.15
        self.Ki = 0.002
        self.actual_speed = 0
        self.errorSum = 0
        self.range = 50
        self.done_lane_change = True

        # ROS init
        os.system('roscore &')

        self.pub_localization = rospy.Publisher('/t4ac/localization/pose',Odometry,queue_size=10)
        self.pub_lane_change = rospy.Publisher('/t4ac/decision_making/lane_change', Bool, queue_size=10)
        rospy.Subscriber("/t4ac/control/cmd_vel", CarControl, self.control_cb)
        rospy.Subscriber("/t4ac/control/lane_change_done", Bool, self.lane_change_cb)

        self.lane_change_msg = Bool()
        self.lane_change_msg.data = False

        rospy.init_node('env_node', anonymous=True)
  
        os.system('roslaunch t4ac_utils_ros t4ac_config.launch &')   
        time.sleep(10)

        self.localization_pub()

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        self.world.apply_settings(settings)

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

        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=-cmd_vel.steer, brake=brake))
        self.world.tick()
        self._move_spectator()

    def lane_change_cb(self, done):
        self.done_lane_change = done.data

    def _move_spectator(self):
        x = self.ego_vehicle.get_transform().location.x
        y = self.ego_vehicle.get_transform().location.y
        world_transform = carla.Transform(carla.Location(x=x, y=y-10, z=15), carla.Rotation(yaw=90, pitch = -50))
        self.world.get_spectator().set_transform(world_transform)
        self.world.tick()

    def _overtake(self):
        vehicles = self.world.get_actors().filter('vehicle.*.*')
        for vehicle in vehicles:
            if vehicle.type_id != 'vehicle.audi.a2' and self.vehicle_in_range(vehicle):
                x_adv, y_adv = self.local_to_global(vehicle)
                if y_adv < 1 and y_adv > -1 and x_adv < 40 and x_adv > 0 and self.done_lane_change:
                    self.done_lane_change = False
                    self.lane_change_msg.data = True
                    self.pub_lane_change.publish(self.lane_change_msg)
                elif self.done_lane_change and self.lane_change_msg.data == True: 
                    self.lane_change_msg.data = False
                    self.pub_lane_change.publish(self.lane_change_msg)


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

if __name__=="__main__": 
    agent = AutonomousAgent()

    while not rospy.is_shutdown():
        agent.localization_pub()
        agent._overtake()
        time.sleep(0.05)

