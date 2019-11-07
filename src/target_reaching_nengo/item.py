#!/usr/bin/env python

class Item(object):
    def __init__(self, name, topic):
        self.position           = None
        self.name               = name
        self.topic              = topic
        self.polar_pos          =  [0.0, 0.0, 0.0] # r, theta, phi of position ModelStates of TCP
        self.vector_to_shoulder = None
