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

        #self.has_joint_state = False
        #self.pose_reference_frame = rospy.get_param('~pose_reference_frame', 'table_corner_link')
        #self.arm_manager = ArmManager(self.pose_reference_frame)
        #rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        #self.data_pub = rospy.Publisher('/data_pub', String, queue_size=1)

    #def joint_states_callback(self, joint_state):
        #self.last_joint_state = list(joint_state.position[0:len(self.arm_manager.joint_names)])
        #if not self.has_joint_state:
            #self.has_joint_state = True

    #def send_arm_trajectory(self, joint_names, positions, duration=1):
        #if not self.has_joint_state:
            #return
        ##self.arm_manager.arm_client.cancel_all_goals()
        #self.arm_manager.new_traj(trajectory_type=RawTrajectory.TT_JOINT_BASED)
        #path = RawTrajectory()
        #path.joint_names = self.arm_manager.joint_names
        #path.waypoint_types.append(RawTrajectory.TT_JOINT_BASED)
        #path.waypoint_names.append('wp')
        #positions_to_send = self.last_joint_state
        #for i in range(4,len(joint_names)):
            #positions_to_send[i] = 0.
        #positions_to_send[1] = 0.5
        #for i in range(1, 4):
            #for j in range(len(joint_names)):
                #if str(i) in joint_names[j]:
                    #if i == 1:
                        #positions_to_send[i-1] = -positions[j]
                    #else:
                        #positions_to_send[i-1] = positions[j]
        #path.positions.append(Float64Array(data=positions_to_send))
        #path.name = 'programmed'
        #path.reference_frame = self.arm_manager.robot_base_link
        #path.type = RawTrajectory.TT_JOINT_BASED

        #self.arm_manager.append_point(0, path)
        #self.arm_manager.arm_client.send_goal(PlayTrajectoryGoal())
        #self.arm_manager.arm_client.wait_for_result()
        #rospy.sleep(1.0)

        #to_pub = String()
        #s = ''
        #for i in range(len(joint_names)):
            #s += '{}: {} - ({})\n'.format(i, joint_names[i], positions[i])
        #to_pub.data = s
        #self.data_pub.publish(to_pub)

    def publish_topic(self, t, x):
        should_publish = [False]*len(self._joints_pub[1])
        if self.use_stim:
            for i in range(len(self._joints_pub[1])):
                self._joints_pub[1][i].publish(x[i])
                should_publish[i] = True
        else:  # FOR TR
            for i in range(len(self._joints_pub[1])):
                # NEAR FAR
                if self._joints_pub[0][i] == '/HoLLiE/hollie_real_left_arm_3_joint/cmd_pos':
                   # self._joints_pub[1][i].publish(-1.1)
                    if abs(self.error[0]) >= 1:
                        self._joints_pub[1][i].publish(x[i])
                        should_publish[i] = True
                # UP DOWN
                elif self._joints_pub[0][i] == '/HoLLiE/hollie_real_left_arm_2_joint/cmd_pos':
                    #self._joints_pub[1][i].publish(1.0)
                    if abs(self.error[1]) >= 1:
                        self._joints_pub[1][i].publish(x[i])
                        should_publish[i] = True
                else:
                    #self._joints_pub[1][i].publish(0.0)
                    if abs(self.error[2]) >= 1:
                        self._joints_pub[1][i].publish(x[i])
                        should_publish[i] = True
        #joint_names = []
        #positions = []
        #for i in range(len(self._joints_pub[1])):
            #if should_publish[i]:
                #joint_names.append(self._joints_pub[0][i])
                #positions.append(x[i])
        #self.send_arm_trajectory(joint_names, positions)


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
                for i in range(len(x)):
                    if self.all_joints[0].count(self.all_joints[i]) == 1:
                        duplette.append([x[i], self.all_joints[i]])
                    else:
                        res.append([x[i], self.all_joints[i]])
                val = (duplette[0][0] +  duplette[1][0]) / 2
                #######  BLEN ERROR #####
                ##### DEBUG #####
                #val = duplette[1][0]
                res.append([val, duplette[0][1]])
                for i in range(len(self._joints_pub[0])):
                    for j in range(len(res)):
                        if self._joints_pub[0][i] == res[j][1]:
                            tmp.append(res[j][0])
                return tmp


            if self.use_stim:
                if self.stim is None: net.stim = nengo.Node(np.zeros(self.slider_nr))
                else: net.stim = nengo.Node(self.stim, label = 'vol __ a __ b')

         #   net.input = nengo.Node(self.set_error)
           # net.f_u= nengo.Ensemble(n_neurons=800, dimensions=len(self.all_joints), radius=14)   #orginal
            net.f_u= nengo.Ensemble(n_neurons=100, dimensions=len(self.all_joints), radius=2, neuron_type=nengo.Direct(), label ='g(f(u))')   #direct
            #net.f_u= nengo.Ensemble(n_neurons=800, dimensions=len(self.all_joints), radius=2)  #ohne direct

            if len(self._joints_pub[0]) is not len(self.all_joints):
                #net.f_u_blended = nengo.Ensemble(n_neurons=700, dimensions=len(set(self._joints_pub[0])), radius=12)   #orginal
                net.f_u_blended = nengo.Ensemble(n_neurons=100, dimensions=len(set(self._joints_pub[0])), radius=2, neuron_type=nengo.Direct(), label= 'g(f(u)) blended')    #direct
                #net.f_u_blended = nengo.Ensemble(n_neurons=800, dimensions=len(set(self._joints_pub[0])), radius=2)     #ohne direct
                nengo.Connection(net.f_u, net.f_u_blended, function= blend)
                net.ros_out = nengo.Node(self.publish_topic, size_in=len(self._joints_pub[0]) )
                nengo.Connection(net.f_u_blended, net.ros_out)#, synapse=0.1)
            else:
                net.ros_out = nengo.Node(self.publish_topic, size_in=len(self._joints_pub[0]))
                nengo.Connection(net.f_u, net.ros_out)#, synapse=0.1)
                # N = 250, radius=np.sqrt(3), synapse = 0.1
        return net
