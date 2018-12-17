import nengo
import numpy as np
import voluntary
import base_network
import error
import feedback
import rospy
from gazebo_msgs.msg import LinkStates
import sys

# TODO: [camilo 2018.12.] ugly hack, this is horrible, change this
sys.path.append('/home/milo/workspace/catkin_ws/src/target_reaching_nengo/scripts')
# sys.path.append('/home/steffen/Schreibtisch/ve_nengo/snn_nengo_motion/src/nengo_ros/src_lea/scripts')
import generateCSV_data

# Variable fuer NRP
#robot = 'robot'
robot = 'HoLLiE'
voluntary_joints = [['/' + robot + '/hollie_real_left_arm_2_joint/cmd_pos'], ['/' + robot + '/hollie_real_left_arm_1_joint/cmd_pos'], ['/' + robot + '/hollie_real_left_arm_2_joint/cmd_pos' , '/' + robot + '/hollie_real_left_arm_3_joint/cmd_pos']]
base_network     = base_network.Base_network(voluntary_joints = voluntary_joints, use_stim = False)
neuron_number = 21

# motion primitives
hoch_runter     = voluntary.Voluntary(slider = 0, joints = voluntary_joints[0], start = [-0.5], end = [2.0], label= 'hoch_runter', neuron_number = neuron_number)#[2.0510], end = [-0.4106])
links_rechts    = voluntary.Voluntary(slider = 1, joints = voluntary_joints[1], start = [-1.7], end = [1.5], label= 'links_rechts', neuron_number = neuron_number) #  start = [-1.5948], end = [1.4762])

#ORGINAL
#fern_nah        = voluntary.Voluntary(slider = 2, joints = voluntary_joints[2], start = [0.47, -2.36], end = [1.25, -1.05],  label= 'fern_nah', joint_mapping = [0.9, 2.0], neuron_number = neuron_number)
fern_nah          = voluntary.Voluntary(slider = 2, joints = voluntary_joints[2], start =  [0.5, -2.3], end =[1.4, -0.2],  label= 'fern_nah_0', neuron_number = neuron_number)
#fern_nah          = voluntary.Voluntary(slider = 2, joints = voluntary_joints[2], start =  [0.5, -2.3], end =[1.0, -0.2],  label= 'fern_nah_0', neuron_number = neuron_number)

#error_1 = error.Error(subject_name = 'unit_sphere_1', threshold = [ [-0.03, 0.03],  [-0.05, 0.05], [-0.05, 0.05]], robot = robot)#threshold = [[-0.05, 0.05], [-0.1, 0.1], [-0.1, 0.1]])#threshold = [[-0.15, 0.15], [-0.15, 0.15], [-0.17, 0.17]]) # TRYOUT: , threshold = [fn, hr, lr])
err = 0.05
error_1 = error.Error(subject_name = 'unit_sphere_1', threshold = [ [-err, err],  [-err, err], [-err, err]], robot = robot)#threshold = [[-0.05, 0.05], [-0.1, 0.1], [-0.1, 0.1]])#threshold = [[-0.15, 0.15], [-0.15, 0.15], [-0.17, 0.17]]) # TRYOUT: , threshold = [fn, hr, lr])

feedback = feedback.Feedback(neuron_number = neuron_number)
sub = rospy.Subscriber('/gazebo/link_states', LinkStates, error_1.callback, queue_size=1)
csv_x = generateCSV_data.GenerateCSV_data(file_name ='x')
csv_y = generateCSV_data.GenerateCSV_data(file_name ='y')
csv_z = generateCSV_data.GenerateCSV_data(file_name ='z')
csv_subject_x = generateCSV_data.GenerateCSV_data(file_name ='subject_x')
csv_subject_y = generateCSV_data.GenerateCSV_data(file_name ='subject_y')
csv_subject_z = generateCSV_data.GenerateCSV_data(file_name ='subject_z')
csv_error_LR = generateCSV_data.GenerateCSV_data(file_name ='left_right')
csv_error_UD = generateCSV_data.GenerateCSV_data(file_name ='up_down')
csv_error_NF = generateCSV_data.GenerateCSV_data(file_name ='near_far')

def get_val(t):
    return error_1.error

def get_tcp_pos_x(t):
    return error_1.tcp.position.x

def get_tcp_pos_y(t):
    return error_1.tcp.position.y

def get_tcp_pos_z(t):
    return error_1.tcp.position.z

def get_subject_pos_x(t):
    return error_1.subject.position.x

def get_subject_pos_y(t):
    return error_1.subject.position.y

def get_subject_pos_z(t):
    return error_1.subject.position.z

model = nengo.Network()
with model:

    net = base_network.get_network('net')
    with net:
        error_node = nengo.Node(get_val)
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
        nengo.Connection(net_fern_nah.output[0], net.f_u[fern_nah._slider])
        nengo.Connection(net_fern_nah.output[1], net.f_u[fern_nah._slider+1])

        #VOLUNTARY_2 : hoch runter  --> fehler theta(1)
        net_hoch_runter = hoch_runter.get_network_TR(label= 'up_down')
        nengo.Connection(error_node[1], net_hoch_runter.input[0], function=base_network.set_errorHR)
        nengo.Connection(net_hoch_runter.output, net.f_u[hoch_runter._slider])

        #VOLUNTARY_2 : links_rechts  --> fehler phi(2)
        net_links_rechts = links_rechts.get_network_TR(label= 'left_right')
        nengo.Connection(error_node[2], net_links_rechts.input[0], function=base_network.set_errorLR)
        nengo.Connection(net_links_rechts.output, net.f_u[links_rechts._slider])






        #Propioception
        net_feedback_joint3_NF = feedback.get_network_position(label= 'FB: near_far', joint= 3, max_val= 0, min_val= -130)
        net_feedback_joint2_HR_NF = feedback.get_network_position(label= 'FB: up_down and near_far' , joint= 2, max_val= 146, min_val= -18)
        net_feedback_joint1_LR = feedback.get_network_position(label= 'FB: left_right', joint= 1, max_val= 74, min_val= -100)


        nengo.Connection(net_feedback_joint3_NF.output, net_fern_nah.input[1])
        #TODO auch net_feedback_joint2_HR_NF an fn verbinden
        nengo.Connection(net_feedback_joint2_HR_NF.output, net_hoch_runter.input[1])
        nengo.Connection(net_feedback_joint1_LR.output, net_links_rechts.input[1])





         # plot x
        net.do_save = nengo.Node(0)
        #TCP
        save_x = nengo.Node(csv_x.csv_save, size_in =2)
        nengo.Connection(net.do_save, save_x[0])
        net.tcp_x_node = nengo.Node(get_tcp_pos_x)
        nengo.Connection(net.tcp_x_node, save_x[1])

        save_y = nengo.Node(csv_y.csv_save, size_in =2)
        nengo.Connection(net.do_save, save_y[0])
        net.tcp_y_node = nengo.Node(get_tcp_pos_y)
        nengo.Connection(net.tcp_y_node, save_y[1])

        save_z = nengo.Node(csv_z.csv_save, size_in =2)
        nengo.Connection(net.do_save, save_z[0])
        net.tcp_z_node = nengo.Node(get_tcp_pos_z)
        nengo.Connection(net.tcp_z_node, save_z[1])

        #subject
        save_subject_x = nengo.Node(csv_subject_x.csv_save, size_in =2)
        nengo.Connection(net.do_save, save_subject_x[0])
        net.subject_x_node = nengo.Node(get_subject_pos_x)
        nengo.Connection(net.subject_x_node, save_subject_x[1])

        save_subject_y = nengo.Node(csv_subject_y.csv_save, size_in =2)
        nengo.Connection(net.do_save, save_subject_y[0])
        net.subject_y_node = nengo.Node(get_subject_pos_y)
        nengo.Connection(net.subject_y_node, save_subject_y[1])

        save_subject_z = nengo.Node(csv_subject_z.csv_save, size_in =2)
        nengo.Connection(net.do_save, save_subject_z[0])
        net.subject_z_node = nengo.Node(get_subject_pos_z)
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

