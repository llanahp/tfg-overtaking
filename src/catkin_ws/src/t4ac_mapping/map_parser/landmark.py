#!/usr/bin/env python3
"""
================
Landmark Class
================
Signals and Objects defined in the OpenDRIVE file are translated as landmark objects that can be queried from the API

Authors: Alejandro D. and J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
"""
# General imports


# T4AC imports
from .builder_classes import T4ac_Transform, T4ac_Location, T4ac_Rotation

class T4ac_LandmarkPose:
    """
    T4ac_Landmark's pose with respect to the of with respect to affecting road.
    """
    def __init__(self, s, t, z_offset, h_offset):
        self.s = s # (float - meters)
        self.t = t # (float - meters)
        self.z_offset = z_offset  # (float - meters)
        self.h_offset = h_offset  # (float - meters)

class T4ac_LandmarkRoad:
    """
    T4ac_LandmarkRoad is used to define each of the roads that are affected by a T4ac_Landmark.
    """
    def __init__(self):
        self.road_id = None # (int)
        self.pose = None # T4ac_LandmarkPose
        self.orientation = None # can be positive (¿??), negative (¿??) or both (affects in both directions of the road)
        self.lanes = [] # (int) if none afecta a todos

class T4ac_Landmark:
    """
    For initializing a landmark in a specific position, the location
    parameter must be passed in T4ac_Location format. If not, other option
    is not passing any parameter an set the parameters after initializing the
    T4ac_Landmark object.
    """
    def __init__(self, location=T4ac_Location("", "", ""), 
                       rotation=T4ac_Rotation("", "", "")):
        self.id = "" # (int)
        self.name = "" # Name of the landmark in OpenDRIVE file (str) 
        self.is_dynamic = "" # (bool)
        self.height = ""  # (float - meters)
        self.width = ""  # (float - meters)
        self.length = ""  # (float - meters)
        self.roll = ""  # (float - rad)
        self.pitch = "" # (float - rad)
        #self.country = None # (str) Country code where the landmark is defined (default to OpenDRIVE is Germany 2017)
            ## At the moment, Roadrunner and Opendrive do not have these types well defined. 
            ## Therefore, we create our own types for the landmarks.
        
        self.type = "" # (str) Type identificator of the landmark according to the country code
        self.transform = T4ac_Transform(location, rotation)
        self.affecting_roads = [] # T4ac_LandmarkRoad()

    