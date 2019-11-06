#!/usr/bin/env python

import nengo

class Item(object):
    
    def __init__(self, name, topic):
        self.position           = None
        self.name               = name
        self.topic              = topic
        self.polar_pos          =  [0.0, 0.0, 0.0] # r, theta, phi of position ModelStates of TCP
        self.vector_to_shoulder = None
        
    #def callback(self, data):
      #  self.position = data.pose[data.name.index(self.topic)].position
        
model = nengo.Network()
