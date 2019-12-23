#!/usr/bin/env python

import nengo
import numpy as np

from target_reaching_nengo import Voluntary, Base_network, Error, Feedback, ErrorTF

import rospy
from gazebo_msgs.msg import LinkStates
import sys
import generateCSV_data

class Main_TR_CL:
    def __init__(self):

        # Variable fuer NRP
        #robot = 'robot'
        #robot = 'HoLLiE'
        robot = 'hbp'
        voluntary_joints = [['/' + robot + '/arm_2_joint/cmd_pos'],
                            ['/' + robot + '/arm_1_joint/cmd_pos'],
                            ['/' + robot + '/arm_2_joint/cmd_pos' ,
                             '/' + robot + '/arm_3_joint/cmd_pos']]
                            #['/' + robot + '/arm_3_joint/cmd_pos']]
        base_network     = Base_network(voluntary_joints = voluntary_joints, use_stim = False)
        neuron_number    = 21

        # motion primitives
        # STANDARD 
        #hoch_runter     = Voluntary(slider = 0, joints = voluntary_joints[0], start = [-0.5], end = [2.0], label= 'hoch_runter', neuron_number = neuron_number)#[2.0510], end = [-0.4106])
        #links_rechts    = Voluntary(slider = 1, joints = voluntary_joints[1], start = [-1.7], end = [1.5], label= 'links_rechts', neuron_number = neuron_number) #  start = [-1.5948], end = [1.4762])
        #fern_nah        = Voluntary(slider = 2, joints = voluntary_joints[2], start =  [-0.5, -2.3], end =[2.0, -0.2],  label= 'fern_nah_0', neuron_number = neuron_number)
        #fern_nah        = Voluntary(slider = 2, joints = voluntary_joints[2], start =  [-2.3], end =[-0.2],  label= 'fern_nah_0', neuron_number = neuron_number)

        #HBP VERSION desired adjusted to standard near_far
        #hoch_runter     = Voluntary(slider = 0, joints = voluntary_joints[0], start = [-0.5], end = [2.0], label= 'hoch_runter', neuron_number = neuron_number)#[2.0510], end = [-0.4106])
        #links_rechts    = Voluntary(slider = 1, joints = voluntary_joints[1], start = [-2.8], end = [-1.5], label= 'links_rechts', neuron_number = neuron_number) #  start = [-1.5948], end = [1.4762])
        #fern_nah          = Voluntary(slider = 2, joints = voluntary_joints[2], start =  [-0.5, -2.3], end =[2.0, -0.2],  label= 'fern_nah_0', neuron_number = neuron_number)

        # everywhere
        hoch_runter     = Voluntary(slider = 0, joints = voluntary_joints[0], start = [-2.8], end = [2.8], label= 'hoch_runter', neuron_number = neuron_number)#[2.0510], end = [-0.4106])
        links_rechts    = Voluntary(slider = 1, joints = voluntary_joints[1], start = [-2.8], end = [2.8], label= 'links_rechts', neuron_number = neuron_number) #  start = [-1.5948], end = [1.4762])
        fern_nah          = Voluntary(slider = 2, joints = voluntary_joints[2], start =  [-2.8, -2.8], end =[2.8, 2.8],  label= 'fern_nah_0', neuron_number = neuron_number)


        # HBP VERSION 1 desired
        #hoch_runter     = Voluntary(slider = 0, joints = voluntary_joints[0], start = [-1.5], end = [0.0], label= 'hoch_runter', neuron_number = neuron_number)#[2.0510], end = [-0.4106])
        #links_rechts    = Voluntary(slider = 1, joints = voluntary_joints[1], start = [0.4], end = [1.4], label= 'links_rechts', neuron_number = neuron_number) #  start = [-1.5948], end = [1.4762])
        #fern_nah          = Voluntary(slider = 2, joints = voluntary_joints[2], start =  [-1.5, -0.3], end =[0.0, 2.2],  label= 'fern_nah_0', neuron_number = neuron_number)

        # HBP VERSION 2
        #hoch_runter     = Voluntary(slider = 0, joints = voluntary_joints[0], start = [-0.5], end = [2.0], label= 'hoch_runter', neuron_number = neuron_number)#[2.0510], end = [-0.4106])
        #links_rechts    = Voluntary(slider = 1, joints = voluntary_joints[1], start = [-2.9], end = [0.1], label= 'links_rechts', neuron_number = neuron_number) #  start = [-1.5948], end = [1.4762])
        #fern_nah          = Voluntary(slider = 2, joints = voluntary_joints[2], start =  [0.5, -2.3], end =[1.4, -0.2],  label= 'fern_nah_0', neuron_number = neuron_number)

        # HBP VERSION 3 links
        #hoch_runter     = Voluntary(slider = 0, joints = voluntary_joints[0], start = [-0.5], end = [2.0], label= 'hoch_runter', neuron_number = neuron_number)#[2.0510], end = [-0.4106])
        #links_rechts    = Voluntary(slider = 1, joints = voluntary_joints[1], start = [-0.18], end = [2.4], label= 'links_rechts', neuron_number = neuron_number) #  start = [-1.5948], end = [1.4762])
        #fern_nah          = Voluntary(slider = 2, joints = voluntary_joints[2], start =  [0.5, -2.3], end =[1.4, -0.2],  label= 'fern_nah_0', neuron_number = neuron_number)
        
        # HBP VERSION 4
        #hoch_runter     = Voluntary(slider = 0, joints = voluntary_joints[0], start = [-0.5], end = [2.0], label= 'hoch_runter', neuron_number = neuron_number)#[2.0510], end = [-0.4106])
        #links_rechts    = Voluntary(slider = 1, joints = voluntary_joints[1], start = [-1.9], end = [1.3], label= 'links_rechts', neuron_number = neuron_number) #  start = [-1.5948], end = [1.4762])
        #fern_nah          = Voluntary(slider = 2, joints = voluntary_joints[2], start =  [0.5, -2.3], end =[1.4, -0.2],  label= 'fern_nah_0', neuron_number = neuron_number)

        #ORGINAL
        #fern_nah        = Voluntary(slider = 2, joints = voluntary_joints[2], start = [0.47, -2.36], end = [1.25, -1.05],  label= 'fern_nah', joint_mapping = [0.9, 2.0], neuron_number = neuron_number)
        #fern_nah          = Voluntary(slider = 2, joints = voluntary_joints[2], start =  [0.5, -2.3], end =[1.4, -0.2],  label= 'fern_nah_0', neuron_number = neuron_number)
        #fern_nah          = Voluntary(slider = 2, joints = voluntary_joints[2], start =  [0.5, -2.3], end =[1.0, -0.2],  label= 'fern_nah_0', neuron_number = neuron_number)

        #self.error_1 = Error(subject_name = 'unit_sphere_1', threshold = [ [-0.03, 0.03],  [-0.05, 0.05], [-0.05, 0.05]], robot = robot)#threshold = [[-0.05, 0.05], [-0.1, 0.1], [-0.1, 0.1]])#threshold = [[-0.15, 0.15], [-0.15, 0.15], [-0.17, 0.17]]) # TRYOUT: , threshold = [fn, hr, lr])
        
        print("[[start],[end]]] lf j1: [{}, {}], ud j2: [{}, {}], nf (j2+)j3: [{}, {}]".format(links_rechts._start, links_rechts._end, hoch_runter._start, hoch_runter._end, fern_nah._start, fern_nah._end))
        
        err = 0.05
        self.error_1 = ErrorTF(subject_name = 'unit_sphere_1', threshold = [ [-err, err],  [-err, err], [-err, err]], robot = robot)#threshold = [[-0.05, 0.05], [-0.1, 0.1], [-0.1, 0.1]])#threshold = [[-0.15, 0.15], [-0.15, 0.15], [-0.17, 0.17]]) # TRYOUT: , threshold = [fn, hr, lr])

        feedback = Feedback(neuron_number = neuron_number)
        sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.error_1.callback, queue_size=1)
        csv_x = generateCSV_data.GenerateCSV_data(file_name ='x')
        csv_y = generateCSV_data.GenerateCSV_data(file_name ='y')
        csv_z = generateCSV_data.GenerateCSV_data(file_name ='z')
        csv_subject_x = generateCSV_data.GenerateCSV_data(file_name ='subject_x')
        csv_subject_y = generateCSV_data.GenerateCSV_data(file_name ='subject_y')
        csv_subject_z = generateCSV_data.GenerateCSV_data(file_name ='subject_z')
        csv_error_LR = generateCSV_data.GenerateCSV_data(file_name ='left_right')
        csv_error_UD = generateCSV_data.GenerateCSV_data(file_name ='up_down')
        csv_error_NF = generateCSV_data.GenerateCSV_data(file_name ='near_far')

        self.model = nengo.Network()
        with self.model:

            net = base_network.get_network('net')
            with net:
                error_node = nengo.Node(self.get_val)
                def kill_error(x):
                    if x==1:
                        return 1.
                    else:
                        return 0.
                def mult(x):
                    return x[0] * x[1] * 2

                #VOLUNTARY_1 : fern nah   --> fehler r (0)
                net_fern_nah = fern_nah.get_network_TR(label= 'near_far')
                nengo.Connection(error_node[0], net_fern_nah.input[0], function=base_network.set_errorFN)
                
                # f_u 4 ouputs
                nengo.Connection(net_fern_nah.output[0], net.f_u[fern_nah._slider])
                nengo.Connection(net_fern_nah.output[1], net.f_u[fern_nah._slider+1])
                
                # f_u 3 outputs
                #nengo.Connection(net_fern_nah.output, net.f_u[fern_nah._slider])

                #VOLUNTARY_2 : hoch runter  --> fehler theta(1)
                net_hoch_runter = hoch_runter.get_network_TR(label= 'up_down')
                nengo.Connection(error_node[1], net_hoch_runter.input[0], function=base_network.set_errorHR)
                nengo.Connection(net_hoch_runter.output, net.f_u[hoch_runter._slider])

                #VOLUNTARY_2 : links_rechts  --> fehler phi(2)
                net_links_rechts = links_rechts.get_network_TR(label= 'left_right')
                nengo.Connection(error_node[2], net_links_rechts.input[0], function=base_network.set_errorLR)
                nengo.Connection(net_links_rechts.output, net.f_u[links_rechts._slider])






                #Propioception ORIGINAL
                #net_feedback_joint3_NF = feedback.get_network_position(label= 'FB: near_far', joint= 3, max_val= 0, min_val= -130)
                #net_feedback_joint2_HR_NF = feedback.get_network_position(label= 'FB: up_down and near_far' , joint= 2, max_val= 146, min_val= -18)
                #net_feedback_joint1_LR = feedback.get_network_position(label= 'FB: left_right', joint= 1, max_val= 74, min_val= -100)
                
                #Propioception ADJUSTED TO START_END STANDARD
                #net_feedback_joint3_NF = feedback.get_network_position(label= 'FB: near_far', joint= 3, max_val= -11, min_val= -130)
                #net_feedback_joint2_HR_NF = feedback.get_network_position(label= 'FB: up_down and near_far' , joint= 2, max_val= 114, min_val= -28)
                #net_feedback_joint1_LR = feedback.get_network_position(label= 'FB: left_right', joint= 1, max_val= 85, min_val= -97)
                
                #Propioception ADJUSTED TO HBP VERSION and near_far
                #net_feedback_joint3_NF = feedback.get_network_position(label= 'FB: near_far', joint= 3, max_val= -11, min_val= -130)
                #net_feedback_joint2_HR_NF = feedback.get_network_position(label= 'FB: up_down and near_far' , joint= 2, max_val= 114, min_val= -28)
                #net_feedback_joint1_LR = feedback.get_network_position(label= 'FB: left_right', joint= 1, max_val= -85, min_val= -160)

                # everywhere
                net_feedback_joint3_NF = feedback.get_network_position(label= 'FB: near_far', joint= 3, max_val= 160, min_val= -160)
                net_feedback_joint2_HR_NF = feedback.get_network_position(label= 'FB: up_down and near_far' , joint= 2, max_val= 160, min_val= -160)
                net_feedback_joint1_LR = feedback.get_network_position(label= 'FB: left_right', joint= 1, max_val= 160, min_val= -160)

                #Propioception ADJUSTED TO HBP VERSION 1 desired
                #net_feedback_joint3_NF = feedback.get_network_position(label= 'FB: near_far', joint= 3, max_val= 126, min_val= -17)
                #net_feedback_joint2_HR_NF = feedback.get_network_position(label= 'FB: up_down and near_far' , joint= 2, max_val= 0, min_val= -85)
                #net_feedback_joint1_LR = feedback.get_network_position(label= 'FB: left_right', joint= 1, max_val= 80, min_val= 22)
                

                nengo.Connection(net_feedback_joint3_NF.output, net_fern_nah.input[1])
                #TODO auch net_feedback_joint2_HR_NF an fn verbinden
                nengo.Connection(net_feedback_joint2_HR_NF.output, net_hoch_runter.input[1])
                nengo.Connection(net_feedback_joint1_LR.output, net_links_rechts.input[1])





                # plot x
                net.do_save = nengo.Node(0)
                #TCP
                save_x = nengo.Node(csv_x.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_x[0])
                net.tcp_x_node = nengo.Node(self.get_tcp_pos_x)
                nengo.Connection(net.tcp_x_node, save_x[1])

                save_y = nengo.Node(csv_y.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_y[0])
                net.tcp_y_node = nengo.Node(self.get_tcp_pos_y)
                nengo.Connection(net.tcp_y_node, save_y[1])

                save_z = nengo.Node(csv_z.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_z[0])
                net.tcp_z_node = nengo.Node(self.get_tcp_pos_z)
                nengo.Connection(net.tcp_z_node, save_z[1])

                #subject
                save_subject_x = nengo.Node(csv_subject_x.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_subject_x[0])
                net.subject_x_node = nengo.Node(self.get_subject_pos_x)
                nengo.Connection(net.subject_x_node, save_subject_x[1])

                save_subject_y = nengo.Node(csv_subject_y.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_subject_y[0])
                net.subject_y_node = nengo.Node(self.get_subject_pos_y)
                nengo.Connection(net.subject_y_node, save_subject_y[1])

                save_subject_z = nengo.Node(csv_subject_z.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_subject_z[0])
                net.subject_z_node = nengo.Node(self.get_subject_pos_z)
                nengo.Connection(net.subject_z_node, save_subject_z[1])


                # ERROR

                save_error_LR = nengo.Node(csv_error_LR.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_error_LR[0])
                nengo.Connection(error_node[2], save_error_LR[1])

                save_error_UD = nengo.Node(csv_error_UD.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_error_UD[0])
                nengo.Connection(error_node[1], save_error_UD[1])

                save_error_NF = nengo.Node(csv_error_NF.csv_save, size_in =2)
                nengo.Connection(net.do_save, save_error_NF[0])
                nengo.Connection(error_node[0], save_error_NF[1])

    def get_val(self, t):
        return self.error_1.error

    def get_tcp_pos_x(self, t):
        return self.error_1.tcp.position.x

    def get_tcp_pos_y(self, t):
        return self.error_1.tcp.position.y

    def get_tcp_pos_z(self, t):
        return self.error_1.tcp.position.z

    def get_subject_pos_x(self, t):
        return self.error_1.subject.position.x

    def get_subject_pos_y(self, t):
        return self.error_1.subject.position.y

    def get_subject_pos_z(self, t):
        return self.error_1.subject.position.z

    def get_Network(self):
        return self.model
