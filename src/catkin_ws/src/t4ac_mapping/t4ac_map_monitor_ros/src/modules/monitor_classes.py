"""
Usefull classes for map_monitor. 
Some of them are replicated from ROS t4ac_msgs.
"""

class Node2D:
    """
    2D point object 

    Attributes:
        x: Coordinate x
        y: Coordinate y
    """
    def __init__ (self, x = 0, y = 0):
        self.x = float(x)
        self.y = float(y)
    
    def __str__ (self):
        return ("{}\n"
                "x = {}\n"
                "y = {}"
                .format(self.__class__, self.x, self.y))

class Node3D:
    """
    3D point object 

    Attributes:
        x: Coordinate x
        y: Coordinate y
        z: Coordinate z
    """
    def __init__ (self, x = 0, y = 0, z = 0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __str__ (self):
        return ("{}\n"
                "x = {}\n"
                "y = {}\n"
                "z = {}"
                .format(self.__class__, self.x, self.y, self.z))
    
class Lane:
    """
    Lane defined by right and left way, with an specific role

    Attributes:
        central_way: List of waypoints defining the center of the lane
        left_way: List of waypoints defining left edge of the lane
        right_way: List of waypoints defining right edge of the lane
    """
    def __init__ (self):
        self.central_way = []
        self.left_way = []
        self.right_way = []
        self.role = ""

    def __str__ (self):
        return ("{}\n"
                "central = {}\n"
                "left = {}\n"
                "right = {}\n"
                "role = {}\n"
                .format(self.__class__, self.central_way, self.left_way, 
                self.right_way, self.role))
