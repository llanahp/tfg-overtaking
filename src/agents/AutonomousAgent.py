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
from std_msgs.msg import String
from sensor_msgs.msg import Image

#RL
import git
BASE_DIR = git.Repo(os.path.dirname(os.path.realpath(__file__)), search_parent_directories=True).working_tree_dir
sys.path.append(BASE_DIR + '/src')
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete
from stable_baselines3 import PPO
from scenarios.Intersection.IntersectionOneState import CustomEnv as IntersectionEnv


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

class AutonomousAgent():
    def __init__(self):

        # Init the scenario in CARLA
        town = "Town03"

        # os.system('cd ~/carla/PythonAPI/util/ && python3 config.py -m' + town)
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()

        if town == "Town03":
            world_transform = carla.Transform(carla.Location(x=84, y=-115, z=150), carla.Rotation(yaw=180, pitch = -90))
        elif town == "Town04":
            world_transform = carla.Transform(carla.Location(x=-13.4, y=-110, z=120), carla.Rotation(yaw=180, pitch = -90))
        self.world.get_spectator().set_transform(world_transform)
        self.map = self.world.get_map()
        blueprint_library = self.world.get_blueprint_library()

        # Ego vehicle
        actors = self.world.get_actors().filter('vehicle.*.*')
        if actors:
            for actor in actors:
                if actor.type_id == 'vehicle.audi.a2':
                    self.ego_vehicle = actor
                    break
        else: 
            ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
            if town == "Town03":
                ego_vehicle_transform = carla.Transform(carla.Location(x=84, y=-85, z=10), carla.Rotation(yaw=270))
            elif town == "Town04":
                ego_vehicle_transform = carla.Transform(carla.Location(x=-13.4, y=-232, z=0.2), carla.Rotation(yaw=90)) 
        
            actors = self.world.get_actors().filter('vehicle.*.*')
        
            for _, actor in enumerate(actors):
                    actor.destroy()
            self.ego_vehicle = self.world.spawn_actor(ego_vehicle_bp,ego_vehicle_transform)

            self.collision_sensor = self.world.spawn_actor(blueprint_library.find('sensor.other.collision'),
                                            carla.Transform(), attach_to=self.ego_vehicle)
            self.collision_sensor.listen(lambda event: self.function_handler(event))

        wp = self.map.get_waypoint(self.ego_vehicle.get_location())
        self.ego_lane_id = abs(wp.lane_id)

        # Camera sensor
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        # Modify the attributes of the blueprint to set image resolution and field of view.
        camera_bp.set_attribute('image_size_x', str(540))
        camera_bp.set_attribute('image_size_y', str(540))
        camera_bp.set_attribute('fov', str(80))
        # Provide the position of the sensor relative to the vehicle.
        transform = carla.Transform(carla.Location(x=-3, y=1.5, z=2), carla.Rotation(pitch=-45, yaw=-30))
        # Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
        self.camera = self.world.spawn_actor(camera_bp, transform, attach_to=self.ego_vehicle)

        # Control PI
        self.Kp = 0.15
        self.Ki = 0.002
        self.actual_speed = 0
        self.errorSum = 0

        self.throttle = 0
        self.steer = 0
        self.brake = 0

        self.acc_velocity = 8

        #RL
        intersection_path = os.path.join(BASE_DIR + '/src/Training/Models/IntersectionCarlaFeaturesOne_PPO_')
        self.intersection_env = IntersectionEnv(render=False)
        self.intersection_model = PPO.load(intersection_path, env = self.intersection_env)
        self.inside_intersection = False

        self.action = 0
        self.agents_index = -1
        self.agents = ['Roundabout Agent', 'Roundabout Agent', 'Lane Change Agent', 'Intersection Agent', 'Merge Agent']
        self.actions = ['Drive', 'Stop', 'Change Left', 'Idle', 'Change Right']

        self.lane_change_msg = Bool()
        self.lane_change_msg.data = False
        self.done_lane_change = False
        self.action_msg = String()
        self.rightjustone = False

        # ROS init
        os.system('roscore &')

        self.pub_localization = rospy.Publisher('/t4ac/localization/pose',Odometry,queue_size=10)
        self.pub_camera = rospy.Publisher('/t4ac/camera', Image, queue_size=10)
        self.pub_state = rospy.Publisher('/t4ac/decision_making/state', String, queue_size=10)
        self.pub_action = rospy.Publisher('/t4ac/decision_making/action', String, queue_size=10)
        self.pub_lane_change = rospy.Publisher('/t4ac/decision_making/lane_change', Bool, queue_size=10)
        rospy.Subscriber("/t4ac/control/cmd_vel", CarControl, self.control_cb)
        rospy.Subscriber("/t4ac/control/intersection", Bool, self.intersection_cb)
        rospy.Subscriber("/t4ac/control/lane_change_done", Bool, self.lane_change_cb)
        
        rospy.init_node('env_node', anonymous=True)
  
        os.system('roslaunch t4ac_utils_ros t4ac_config.launch &')   
        time.sleep(10)

        self.time_reset = time.time()
        # with open ('speed.txt', 'w') as file:
        #     file.write("Ego vehicle speed")
        # with open ('time.txt', 'w') as file:
        #     file.write("Time")

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

    def camera_pub(self, image):

        img, encoding = self.get_carla_image_data_array(image)

        img = self.cv2_to_imgmsg(img, encoding)

        camera_msg = Image()
        camera_msg = img
        camera_msg.header.stamp = rospy.Time.now()
        camera_msg.header.frame_id = "camera"

        self.pub_camera.publish(camera_msg)

    def get_carla_image_data_array(self, carla_image):

        carla_image_data_array = np.ndarray(
            shape=(540, 540, 4),
            dtype=np.uint8, buffer=carla_image.raw_data)

        return carla_image_data_array, 'bgra8'

    def cv2_to_imgmsg(self, cvim, encoding = "passthrough"):
        """
        Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::Image message.
        :param cvim:      An OpenCV :cpp:type:`cv::Mat`
        :param encoding:  The encoding of the image data, one of the following strings:
            * ``"passthrough"``
            * one of the standard strings in sensor_msgs/image_encodings.h
        :rtype:           A sensor_msgs.msg.Image message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``
        If encoding is ``"passthrough"``, then the message has the same encoding as the image's OpenCV type.
        Otherwise desired_encoding must be one of the standard image encodings
        This function returns a sensor_msgs::Image message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """

        if not isinstance(cvim, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')
        img_msg = Image()
        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]

        if len(cvim.shape) < 3:
            cv_type = 'mono8' 
        else:
            cv_type = 'bgr8'
        if encoding == "passthrough":
            img_msg.encoding = cv_type
        else:
            img_msg.encoding = encoding

        if cvim.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data = cvim.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height

        return img_msg

    def intersection_cb(self, intersection):
        if intersection.data != self.inside_intersection and intersection.data == True:
            self.agents_index +=1
        if self.agents_index == 2 and self.lane_change_msg.data == False:
            self.lane_change_msg.data = True
            self.pub_lane_change.publish(self.lane_change_msg)
            self.action_msg.data = self.actions[2]
            self.pub_action.publish(self.action_msg)
        self.inside_intersection = intersection.data

    def lane_change_cb(self, done):
        self.done_lane_change = done.data

    def control_cb(self, cmd_vel):
        self.actual_speed = math.sqrt(pow(self.ego_vehicle.get_velocity().x,2)+pow(self.ego_vehicle.get_velocity().y,2))
        if self.acc_velocity < cmd_vel.velocity and self.acc_velocity >= 0: 
            cmd_vel.velocity = self.acc_velocity
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

        self.throttle = throttle
        self.steer = -cmd_vel.steer
        self.brake = brake

        ## Print values for graphs

        # text = "{}\n'".format(self.actual_speed)
        # with open ('speed.txt', 'a') as file:
        #     text = "{}\n'".format(self.actual_speed)
        #     file.write(text)
        # text = "{}\n'".format(-cmd_vel.steer)
        # with open ('steer.txt', 'a') as file:
        #     text = "{}\n'".format(-cmd_vel.steer)
        #     file.write(text)    
        # with open ('time.txt', 'a') as file:
        #     text = "{}'\n'".format(time.time() - self.time_reset)
        #     file.write(text)

    def _move_spectator(self):
        x = self.ego_vehicle.get_transform().location.x
        y = self.ego_vehicle.get_transform().location.y
        world_transform = carla.Transform(carla.Location(x=x+10, y=y, z=25), carla.Rotation(yaw=180, pitch = -50))
        self.world.get_spectator().set_transform(world_transform)
        self.world.tick()

    
    def apply_control(self):
        state_msg = String()
        if not self.inside_intersection:
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=self.throttle, steer=self.steer, brake=self.brake))
            state_msg.data = 'No Agent'
            if self.agents_index == 2:
                state_msg.data = self.agents[self.agents_index]
                self.action_msg.data = self.actions[3]  
                if self.done_lane_change == True and self.ego_vehicle.get_transform().location.y < -60 and self.rightjustone == False:
                    lane_change_msg = Bool()
                    lane_change_msg.data = False
                    self.pub_lane_change.publish(lane_change_msg)
                    self.action_msg.data = self.actions[4]
                    self.rightjustone = True
                
                self.pub_action.publish(self.action_msg)
            self.pub_state.publish(state_msg)
            self._move_spectator()
        elif self.inside_intersection and self.action == 0:
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=self.throttle, steer=self.steer, brake=self.brake))
            state_msg.data = self.agents[self.agents_index]
            self.pub_state.publish(state_msg)
            self.action_msg.data = self.actions[0]
            self.pub_action.publish(self.action_msg)
            self._move_spectator()
        elif self.inside_intersection and self.action == 1:
            while(self.actual_speed > 0):
                state_msg.data = self.agents[self.agents_index]
                self.pub_state.publish(state_msg)
                self.action_msg.data = self.actions[1]
                self.pub_action.publish(self.action_msg)
                self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))
                self._move_spectator()


if __name__=="__main__": 
    agent = AutonomousAgent()
    agent.camera.listen(lambda image: agent.camera_pub(image))
    obs = agent.intersection_env.reset()

    while not rospy.is_shutdown():
        agent.acc_velocity = agent.intersection_env._acc()
        agent.localization_pub()
        agent.apply_control()
        time.sleep(0.05)
        obs, reward, done, info = agent.intersection_env.step(0)
        if agent.inside_intersection:
            agent.action, _ = agent.intersection_model.predict(obs)
            # print("action: ",agent.action)
