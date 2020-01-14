#!/usr/bin/env python

import rospy
from target_reaching_nengo import Main_TR_CL

rospy.init_node('Main_TR_CL_nengo_ros', anonymous=True, disable_signals=True)
main_TR_CL = Main_TR_CL()

model = main_TR_CL.get_Network()
