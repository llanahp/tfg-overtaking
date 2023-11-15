#!/usr/bin/env python
"""
Some classes describing structures to operate with map structure (already parsed from an xodr)

Authors: Alejandro D. and J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
"""

class T4ac_Location:
    def __init__(self, x="", y="", z=""):
        self.x = x  # (float - meters)
        self.y = y # (float - meters)
        self.z = z # (float - meters)

class T4ac_Rotation:
    def __init__(self, pitch="", yaw="", roll=""):
        self.pitch = pitch # (float - degrees)
        self.yaw = yaw # (float - degrees)
        self.roll = roll # (float - degrees)

class T4ac_Transform:
    def __init__(self, location=T4ac_Location("", "", ""), 
                       rotation=T4ac_Rotation("", "", "")):
        self.location = T4ac_Location(location.x, location.y, location.z)
        self.rotation = T4ac_Rotation(rotation.pitch, rotation.yaw, rotation.roll)