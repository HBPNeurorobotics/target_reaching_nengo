# !/usr/bin/env python
import rospy
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
import random
import math
import random
import time
import numpy as np
import generateCSV_data

class service(object):
    def __init__(self, name):
        self.snooze = 4
        self.base_x = 0.024
        self.base_y = 0.518
        self.base_z = 1.119

        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_state = ModelState()
        self.model_state.model_name = name
        self.model_state.pose.orientation.x = 0.0
        self.model_state.pose.orientation.y = 0.0
        self.model_state.pose.orientation.z = 0.0
        self.model_state.pose.orientation.w = 1.0
        self.model_state.scale.x = 1.0
        self.model_state.scale.y = 1.0
        self.model_state.scale.z = 1.0
        self.model_state.twist.linear.z = 0.0
        self.model_state.twist.angular.x = 0.0
        self.model_state.twist.angular.y = 0.0
        self.model_state.twist.angular.z = 0.0
        self.model_state.reference_frame = 'world'

    def set_polar(self,r, theta, phi):
        #cartesian = self.asCartesian([r, theta, phi])
        cartesian_nengo = self.calc_cartesian(r, theta, phi)
        self.set_pos(cartesian_nengo[0], cartesian_nengo[1], cartesian_nengo[2])

    def set_base_polar(self):
        self.set_polar(0.475, 1.535, 4.75)


    def set_base_cartesian(self):
        self.set_pos(1.0, -1.9, 1.0)


    def set_random_cartesian(self):
        self.set_pos(random.uniform(0.5, 1.6), random.uniform(-1.7, -2.1), random.uniform(0.6, 1.6))
        print 'Cartesian: x: ', self.model_state.pose.position.x, 'y: ', self.model_state.pose.position.y, 'z: ', self.model_state.pose.position.z
        self.set_model_state(self.model_state)

    def set_random_polar(self):
        r = random.uniform(0.2, 0.7)  # 0.33, 0.65
        theta = random.uniform(0.5, 2.5)
        phi = random.uniform(3.0, 6.3)  # 3,3
        cartesian = self.asCartesian([r, theta, phi])
        cartesian_nengo = self.calc_cartesian(r, theta, phi)
        self.set_pos(cartesian_nengo[0], cartesian_nengo[1], cartesian_nengo[2])
        print '###########################'
        print 'Polar: r: ', r, '  theta: ', theta, '  phi: ', phi
        print cartesian



    def set_pos(self, x, y, z):
        self.model_state.pose.position.x = x + self.base_x
        self.model_state.pose.position.y = y + self.base_y
        self.model_state.pose.position.z = z + self.base_z
        self.set_model_state(self.model_state)



    def asCartesian(self, rthetaphi):
        r = rthetaphi[0]
        theta = rthetaphi[1] * math.pi / 180  # to radian
        phi = rthetaphi[2] * math.pi / 180
        x = r * math.sin(theta) * math.cos(phi)
        y = r * math.sin(theta) * math.sin(phi)
        z = r * math.cos(theta)
        return [x, y, z]

    def calc_cartesian(self, r, theta, phi):
        x = r * math.sin(theta) * math.cos(phi)
        y = r * math.sin(theta) * math.sin(phi)
        z = r * math.cos(theta)
        return [x, y, z]



    # set to specific point, used to reach NF UD LR
    def set_limits(self):
        # Average: 0.475, 1.535, 4.75

        # r (0.2, 0.6)   Nah fern
        self.set_cartesian(0.2, 1.535, 4.75)
        self.set_cartesian(0.6, 1.535, 4.75)

        # theta (0.5, 2.5)  hoch runter
        self.set_cartesian(0.475, 0.5, 4.75)
        self.set_cartesian(0.475, 2.5, 4.75)


        # phi (3.0, 6.3)    links rechts
        self.set_cartesian(0.475, 1.535, 3.0)
        self.set_cartesian(0.475, 1.535, 6.3)



        # set to specific point, used to reach the 10 points on both inner and outer circle
    def set_ALL_limits(self):
        print '###########################'
        print 'START'
        #print 'NEAR center'
        #self.set_cartesian(0.2, 1.535, 4.75)

        print 'NEAR up'
        self.set_cartesian(0.2, 0.5, 4.75)
        print 'NEAR left'
        self.set_cartesian(0.2, 1.535, 3.0)
        print 'NEAR down'
        self.set_cartesian(0.2, 2.5, 4.75)
        print 'NEAR right'
        self.set_cartesian(0.2, 1.535, 6.3)
        print 'NEAR up'
        self.set_cartesian(0.2, 0.5, 4.75)


        #print 'FAR center'
        #self.set_cartesian(0.6, 1.535, 4.75)

        print 'FAR up'
        self.set_cartesian(0.6, 0.5, 4.75)
        print 'FAR left'
        self.set_cartesian(0.6, 1.535, 3.0)
        print 'FAR left'
        self.set_cartesian(0.6, 2.5, 4.75)
        print 'FAR right'
        self.set_cartesian(0.6, 1.535, 6.3)
        print 'FAR up'
        self.set_cartesian(0.6, 0.5, 4.75)




    def set_cartesian(self, r, theta, phi):
        cartesian_nengo = self.calc_cartesian(r, theta, phi)
        self.set_pos(cartesian_nengo[0], cartesian_nengo[1], cartesian_nengo[2])
        time.sleep(serv_target.snooze)

    # alternates between home position and random position
    def star_random(self):
        self.set_base_polar()
        time.sleep(self.snooze)
        self.set_random_polar()
        time.sleep(self.snooze)

    # alternates between home position and sampling position
    # , , phi in 0.1 steps
    def star_sample(self):
        # r (0.2, 0.6)   Nah fern   sampelt r in 0.05 steps
        for r_ in xrange(4, 13):
            # theta (0.5, 2.5)  hoch runter     sampelt theta in 0.1 steps
            for t_ in xrange(5, 26):
                # phi (3.0, 6.3)    links rechts     sampelt phi in 0.1 steps
                for p_ in xrange(30, 64):
                    r = r_ / float(20)
                    t = t_ / float(10)
                    p = p_ / float(10)
                    self.set_base_polar()
                    time.sleep(self.snooze)
                    self.set_polar(r, t, p)
                    print '###########################'
                    print 'Polar: r: ', r , '  theta: ', t , '  phi: ', p
                    time.sleep(self.snooze)

        print '###########################'
        print ''

    def all_random(self):
        self.set_random_polar()
        time.sleep(self.snooze)

if __name__== '__main__':
    rospy.init_node('pub_random')
    serv_robot = service('HoLLiE')
    serv_robot.set_pos(0.03, 0.5, 0.02)


    while not rospy.is_shutdown():
        rospy.wait_for_service('/gazebo/set_model_state')
        serv_target = service('unit_sphere_1')

        #serv_target.all_random()
        #serv_target.star_sample()
        #serv_target.set_limits()
        serv_target.set_ALL_limits()






