"""
Last edit: 14 / april / 2021 by Alejandro D.
Util functions to work with an xodr map objetc. 
Some of this functions have been adapted from the map_builder class
designed for the carla_challenge to work without the PythonAPI
"""
import sys
import math 


def euclidean_distance(wp1, wp2):
    """
    Computes euclidean distance from one waypoint to another
    Args:
        wp1: (t4ac.Waypoint)
        wp2: (t4ac.Waypoint)
    Returns: 
        distance: (float) Euclidean distance
    """

    x1 = wp1.transform.location.x
    y1 = wp1.transform.location.y
    x2 = wp2.transform.location.x
    y2 = wp2.transform.location.y

    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) **2)

    return distance
