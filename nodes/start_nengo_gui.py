#!/usr/bin/env python

import nengo_gui
import rospkg
import os

file_path = rospkg.RosPack().get_path('target_reaching_nengo')+'/nodes/main_TR_CL_node.py'
command = 'nengo ' + file_path
os.system(command)
