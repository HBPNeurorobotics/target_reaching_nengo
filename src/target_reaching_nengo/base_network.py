#!/usr/bin/env python

import nengo
import numpy as np

import rospy
from std_msgs.msg import Float64, String
from visuomotor_manager import ArmManager
import actionlib

from fzi_manipulation_msgs.msg import RawTrajectory, PlayTrajectoryAction, PlayTrajectoryGoal
from fzi_std_msgs.msg import Float64Array
from sensor_msgs.msg import JointState

class Base_network():
    def __init__(self, voluntary_joints = [],  rhythmic_joints = [], stim = None, use_stim = True):
        self.stim = stim
        self.use_stim = use_stim
        self.slider_nr = len(voluntary_joints) + 2* len(rhythmic_joints)
        self.all_motions = voluntary_joints + rhythmic_joints
        self.all_joints =[]
        self.rate = rospy.Rate(10)
        self.error = [0.0, 0.0, 0.0]
        for i in range(len(self.all_motions)):
            for j in range(len(self.all_motions[i])):
                self.all_joints.append(self.all_motions[i][j])
        unique_joints = sorted(set(self.all_joints), key=self.all_joints.index)
        self._joints_pub= [[] for y in range(2)]
        for i in unique_joints:
            self._joints_pub[0].append(i)
            self._joints_pub[1].append(rospy.Publisher(i, Float64, queue_size=1))
        self.result = []
        self.robot = 'hbp'
        self.near_far_joint_name = '/' + self.robot + '/arm_3_joint/cmd_pos'
        self.up_down_joint_name = '/' + self.robot + '/arm_2_joint/cmd_pos'
        self.last_duplette = None
        self.last_used_index = -1

    def publish_topic(self, t, x):
        if self.use_stim:
            for i in range(len(self._joints_pub[1])):
                self._joints_pub[1][i].publish(x[i])
        else:  # FOR TR(=TARGET REACHING)
            for i in range(len(self._joints_pub[1])):
                # NEAR FAR
                if self._joints_pub[0][i] == self.near_far_joint_name:
                    if abs(self.error[0]) >= 1:
                        self._joints_pub[1][i].publish(x[i])
                # UP DOWN
                elif self._joints_pub[0][i] == self.up_down_joint_name:
                    if abs(self.error[1]) >= 1:
                        self._joints_pub[1][i].publish(x[i])
                # LEFT RIGHT
                else:
                    if abs(self.error[2]) >= 1:
                        self._joints_pub[1][i].publish(x[i])


    def set_start_pos(self, joint, value):
        pub= rospy.Publisher(joint, Float64, queue_size=1)
        pub.publish(value)

    def set_errorFN(self, x):
        self.error[0] = x
        return x

    def set_errorHR(self, x):
        self.error[1] = x
        return x

    def set_errorLR(self, x):
        self.error[2] = x
        return x


    def get_network(self, label):
        net = nengo.Network(label=label)
        with net:
            def blend(x):
                # TODO : wenn mehr als 1 joint doppelt?
                res=[]
                duplette=[]
                tmp=[]
                #print("blend x: {}".format(x))
                for i in range(len(x)):
                    if self.all_joints[0].count(self.all_joints[i]) == 1:
                        duplette.append([x[i], self.all_joints[i]])
                    else:
                        res.append([x[i], self.all_joints[i]])
                # use only changed value
                if (abs(self.error[1]) >= 1 and abs(self.error[0]) < 1) or (abs(self.error[1]) < 1 and abs(self.error[0]) < 1 and self.last_used_index == 0):
                    val = duplette[0][0]
                    self.last_used_index = 0
                elif (abs(self.error[1]) < 1 and abs(self.error[0]) >= 1) or (abs(self.error[1]) < 1 and abs(self.error[0]) < 1 and self.last_used_index == 1):
                    val = duplette[1][0]
                    self.last_used_index = 1
                else:
                    val = (duplette[0][0] +  duplette[1][0]) / 2
                    self.last_used_index = -1
                #print("val: {}, res: {}, duplette: {}".format(val, res, duplette))
                #######  BLEN ERROR #####
                ##### DEBUG #####
                #val = duplette[1][0]
                res.append([val, duplette[0][1]])
                for i in range(len(self._joints_pub[0])):
                    for j in range(len(res)):
                        if self._joints_pub[0][i] == res[j][1]:
                            tmp.append(res[j][0])
                #print("res: {}, tmp: {}".format(res, tmp))
                return tmp

            def id_func(x):
                return x

            if self.use_stim:
                if self.stim is None: net.stim = nengo.Node(np.zeros(self.slider_nr))
                else: net.stim = nengo.Node(self.stim, label = 'vol __ a __ b')

            net.f_u= nengo.Ensemble(n_neurons=100, dimensions=len(self.all_joints), radius=2, neuron_type=nengo.Direct(), label ='g(f(u))')   #direct

            # bijective 3 to 3
            #if len(self._joints_pub[0]) is not len(self.all_joints):
                #net.f_u_blended = nengo.Ensemble(n_neurons=100, dimensions=len(set(self._joints_pub[0])), radius=2, neuron_type=nengo.Direct(), label= 'g(f(u)) blended')    #direct
                #nengo.Connection(net.f_u, net.f_u_blended, synapse=0.001)
                #net.ros_out = nengo.Node(self.publish_topic, size_in=len(self._joints_pub[0]) )
                #nengo.Connection(net.f_u_blended, net.ros_out)#, synapse=0.1)
            #else:
                #net.f_u_blended = nengo.Ensemble(n_neurons=100, dimensions=len(set(self._joints_pub[0])), radius=2, neuron_type=nengo.Direct(), label= 'g(f(u)) blended')    #direct
                #nengo.Connection(net.f_u, net.f_u_blended, function=id_func, synapse=0.001)
                #net.ros_out = nengo.Node(self.publish_topic, size_in=len(self._joints_pub[0]))
                
                
            # only surjective, f_u output number is 4 to 3
            if len(self._joints_pub[0]) is not len(self.all_joints):
                net.f_u_blended = nengo.Ensemble(n_neurons=100, dimensions=len(set(self._joints_pub[0])), radius=2, neuron_type=nengo.Direct(), label= 'g(f(u)) blended')    #direct
                nengo.Connection(net.f_u, net.f_u_blended, function= blend, synapse=0.001)
                net.ros_out = nengo.Node(self.publish_topic, size_in=len(self._joints_pub[0]) )
                nengo.Connection(net.f_u_blended, net.ros_out)
            else:
                net.f_u_blended = nengo.Ensemble(n_neurons=100, dimensions=len(set(self._joints_pub[0])), radius=2, neuron_type=nengo.Direct(), label= 'g(f(u)) blended')    #direct
                nengo.Connection(net.f_u, net.f_u_blended, function=id_func, synapse=0.001)
                net.ros_out = nengo.Node(self.publish_topic, size_in=len(self._joints_pub[0]))
                nengo.Connection(net.f_u_blended, net.ros_out)
                
                
        return net
