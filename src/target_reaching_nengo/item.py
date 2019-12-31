#!/usr/bin/env python

from geometry_msgs.msg import Point

class Item(object):
    def __init__(self, name, topic):
        self.position           = Point()
        self.orientation        = None
        self.name               = name
        self.topic              = topic
        self.polar_pos          =  [0.0, 0.0, 0.0] # r, theta, phi of position ModelStates of TCP
        self.vector_to_shoulder = None
