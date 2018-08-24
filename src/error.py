from __future__ import division
import nengo
import rospy
import numpy as np
import math
import sys, os
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from random import uniform
import copy
import item
sys.path.append('/home/steffen/papers/own/TR_nengo_code/ve_nengo/snn_nengo_motion/src/nengo_ros/src_lea/scripts')
#sys.path.append(os.path.join(sys.path[0],'scripts'))
import generate_curve_data


class Error(object):
    
    def __init__(self, subject_name, threshold, learning = False, n_points = 41, amplitude = 0.2, period = 2 * np.pi, phase_shift = 0, vertical_shift = 0, do_print = False, mult_with_radius = False, robot = 'HoLLiE'):
        self.robot          = robot
        self.subject        = item.Item('subject', subject_name + '::link')
        self.cmd            = item.Item('cmd', 'cmd_TR')
        #state               = LinkStates()
        #self.cmd.position        = state.position
        self.tcp            = item.Item('tcp', robot + '::hollie_real_left_hand_f1_link')
        self.shoulder       = item.Item('shoulder', robot + '::hollie_real_left_arm_0_joint_link')
        self.threshold      = threshold
        
        self.learning       = learning
        self.dif = self.error = [0.0, 0.0, 0.0]
        
        #cartesian learning
        self.tcp_position_z = 0.0
        self.tcp_position_y = 0.0
        self.value_curve = 0.0
        self.curve_index_normed = 0.0
        self.indice_curve = 0.0
        generate_curve_data_1 = generate_curve_data.Generate_curve(n_points, amplitude, period, phase_shift, vertical_shift, do_print)
        self.curve_data  =  generate_curve_data_1.data
        
       
        
        
    def callback(self, data):
        #print 'BLA: ', self.cmd
        self.subject.position   = data.pose[data.name.index(self.subject.topic)].position

        self.tcp.position       = data.pose[data.name.index(self.tcp.topic)].position
        self.shoulder.position  = data.pose[data.name.index(self.shoulder.topic)].position
        # dies zeile an subject fehler schuld, nicht an tcp
        
        
        self.subject.vector_to_shoulder = self.check_limit(self.calc_vector(self.subject.position))
        self.tcp.vector_to_shoulder = self.calc_vector(self.tcp.position)
        self.subject.polar_pos  =  self.calc_polar(self.subject.vector_to_shoulder.x, self.subject.vector_to_shoulder.y, self.subject.vector_to_shoulder.z)
        self.tcp.polar_pos      =   self.calc_polar(self.tcp.vector_to_shoulder.x, self.tcp.vector_to_shoulder.y, self.tcp.vector_to_shoulder.z)
        self.dif = self.calc_dif()
        #print 'POLAR: ', self.tcp.polar_pos
        if self.learning: self.error = self.calc_error_2()
        else: self.error = self.calc_error()
        
      
    
    #  um aus main klassen error zu bekommen 
    def get_val(self, t):
        return self.error
        
    #  um aus main klassen die position  von tcp zu bekommen 
    def get_tcp(self, t):
        pos = self.tcp.position
        if pos is not None: return [pos.x, pos.y, pos.z]  
        else: return [0.0, 0.0, 0.0]
        
    #  um aus main klassen die position von subject/ shoulder zu bekommen 
    def get_subject(self, t):
        pos = self.subject.position
        if pos is not None: return [pos.x, pos.y, pos.z]  
        else: return [0.0, 0.0, 0.0]
        
    #  um aus main klassen die positionen von shoulder zu bekommen 
    def get_shoulder(self, t):
        pos = self.shoulder.position
        res = None
        if self.shoulder.position is not None and self.tcp.position is not None:
            res = [self.shoulder.position.x - self.tcp.position.x, self.shoulder.position.y - self.tcp.position.y, self.shoulder.position.z - self.tcp.position.z]
        if pos is not None: return [pos.x, pos.y, pos.z]  
        else: return [0.0, 0.0, 0.0]
        
        
        
        
      
    # berechnet koordinaten im bezug auf shoulder
    def calc_vector(self, p): 
        res = copy.copy(p)
        res.x = p.x -self.shoulder.position.x
        res.y = p.y -self.shoulder.position.y
        res.z = p.z -self.shoulder.position.z
        return res
        
    def check_limit(self, subject):
        if subject.y > 0.0 : subject.y = -0.0001
        return subject
  
    def calc_polar(self, x, y, z):
        r       = math.sqrt( math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2))
        theta   = math.acos(z / r)
        #phi     = math.atan2(y,x) 
        
        if(y < 0): phi = 2*np.pi - math.acos(x/ math.sqrt(math.pow(x,2) + math.pow(y,2)))  #math.atan2(y,x) 
        else: phi = math.acos(x/ math.sqrt(math.pow(x,2) + math.pow(y,2)))
        
        return [r, theta, phi]
        
    def calc_cartesian(self, r, theta, phi):
        x = r * math.sin(theta) * math.cos(phi)
        y = r * math.sin(theta) * math.sin(phi)
        z = r * math.cos(theta) 
        return[x, y, z]
    
    def calc_dif(self):
        dif = [ self.subject.polar_pos[0] - self.tcp.polar_pos[0],  # r
                self.subject.polar_pos[1] - self.tcp.polar_pos[1],  #theta
                self.subject.polar_pos[2] - self.tcp.polar_pos[2]]  #phi
        return dif
        
     
    def calc_error_old(self):
        error = [0,0,0]
        for i in range(3):
            if abs(self.dif[i]) > self.threshold[i][1]: 
                error[i] = 1
                if abs(self.dif[i]) > self.threshold[i][1]    : error[i] = 1
                if abs(self.dif[i]) > self.threshold[i][1] * 5: error[i] = 2
                if abs(self.dif[i]) > self.threshold[i][1] * 10: error[i] = 3
                error[i] = error[i] * (self.dif[i] / abs(self.dif[i])) 
      
        #print '   '
        #print 'Dif1 fern_nah: ', self.dif[0], ' upper: ', self.threshold[0][1], 'lower: ',  self.threshold[0][0], '  Error: ', error[0]
        #print 'Dif2 hoch runter: ', self.dif[1], ' upper: ', self.threshold[1][1], 'lower: ',  self.threshold[1][0], '  Error: ', error[1]
        #print 'Dif3 links rechts: ', self.dif[2], ' upper: ', self.threshold[2][1], 'lower: ',  self.threshold[2][0], '  Error: ', error[2]
        #print '   '
        
        #print 'ERROR: ', error[2]
        return error
        
        
    def calc_error(self):
        error = [0,0,0]
        for i in range(3):
            if abs(self.dif[i]) > self.threshold[i][1]: 
                error[i] = 1
                if abs(self.dif[i]) > self.threshold[i][1]    : error[i] = 1
                if abs(self.dif[i]) > self.threshold[i][1] * 5: error[i] = 2
                if abs(self.dif[i]) > self.threshold[i][1] * 10: error[i] = 3
                error[i] = error[i] * (self.dif[i] / abs(self.dif[i])) 
            
            if i == 0 and error[i] > 0:
                error[i] = error[i] * 3
      
        #print '   '
        #print 'Dif1 fern_nah: ', self.dif[0], ' upper: ', self.threshold[0][1], 'lower: ',  self.threshold[0][0], '  Error: ', error[0]
        #print 'Dif2 hoch runter: ', self.dif[1], ' upper: ', self.threshold[1][1], 'lower: ',  self.threshold[1][0], '  Error: ', error[1]
        #print 'Dif3 links rechts: ', self.dif[2], ' upper: ', self.threshold[2][1], 'lower: ',  self.threshold[2][0], '  Error: ', error[2]
        #print '   '
        
        #print 'ERROR: ', error[2]
        return error    
   

    def calc_error_fern_nah(self):
        error = [0,0,0]
        for i in range(3):
            # damit fern nah einen starkeren fehler bekommt
            if i == 0:
                if abs(self.dif[i]) > self.threshold[i][1]: 
                    error[i] = 1
                    if abs(self.dif[i]) > self.threshold[i][1]    : error[i] = 2
                    if abs(self.dif[i]) > self.threshold[i][1] * 5: error[i] = 4
                    if abs(self.dif[i]) > self.threshold[i][1] * 10: error[i] = 5
                    error[i] = error[i] * (self.dif[i] / abs(self.dif[i]))
            else:
                if abs(self.dif[i]) > self.threshold[i][1]: 
                    error[i] = 1
                    if abs(self.dif[i]) > self.threshold[i][1]    : error[i] = 1
                    if abs(self.dif[i]) > self.threshold[i][1] * 5: error[i] = 2
                    if abs(self.dif[i]) > self.threshold[i][1] * 10: error[i] = 3
                    error[i] = error[i] * (self.dif[i] / abs(self.dif[i]))
   
        #print '   '
        print 'Dif1 fern_nah: ', self.dif[0], ' upper: ', self.threshold[0][1], 'lower: ',  self.threshold[0][0], '  Error: ', error[0]
        #print 'Dif2 hoch runter: ', self.dif[1], ' upper: ', self.threshold[1][1], 'lower: ',  self.threshold[1][0], '  Error: ', error[1]
        #print 'Dif3 links rechts: ', self.dif[2], ' upper: ', self.threshold[2][1], 'lower: ',  self.threshold[2][0], '  Error: ', error[2]
        #print '   '
        
        #print 'ERROR: ', error[2]
        return error
   
    def calc_error_2(self):
    
        error = [0,0,0]
        for i in range(3):
            if i is 0:
                if abs(self.dif[i]) > self.threshold[i][1]: 
                    error[i] = 5
                    error[i] = error[i] * (self.dif[i] / abs(self.dif[i])) 
      
        return error
   
   
   
   
   ########################################### L E A R N I N G  - C A R T E S I A N #################################
   
   
    def get_tcp_position(self, t):
        return [self.tcp_position_y, self.tcp_position_z] 
        
    def get_value_curve(self, x):
        return [self.indice_curve ,self.value_curve]


    def set_tcp_position(self, x):
        def percentage(part, whole):
            return 100 * float(part)/float(whole)
        
        dist = [0.0, 0.0]
        punkt_1 = copy.copy(self.shoulder.position) # shoulder position
        punkt_2 = copy.copy(self.subject.position)  # subject position
        punkt_3 = copy.copy(self.tcp.position)      # tcp position
        if punkt_1 is not None and punkt_2 is not None and punkt_3 is not None:
            initial_tcp_y  = punkt_1.y +0.3
            initial_tcp_z  = punkt_1.z
            subject_y   = punkt_2.y
            subject_z   = punkt_2.z
            tcp_y       = punkt_3.y
            tcp_z       = punkt_3.z
    
            if initial_tcp_y < 0 or initial_tcp_z < 0 or subject_y < 0 or subject_z < 0 or tcp_y < 0 or tcp_z < 0:
                print 'WARNING: Calculation of error does not work with negative values.'
            else:    
    
                if abs(subject_z - initial_tcp_z) > 0.01: print 'shoulder and subject schould be in one line (same z value)'
                avg_z = (initial_tcp_z + subject_z)/2 
                self.tcp_position_z = tcp_z - avg_z
                length_y =  subject_y - initial_tcp_y
                pos_y = tcp_y - initial_tcp_y
                self.tcp_position_y = max(0.0, min(1.0, percentage(pos_y, length_y) /100))
        
    
    
    def set_value_curve(self, x):
        curve_index_normed = 0.0
        dist = [0.0, 0.0]
        punkt_1 = copy.copy(self.shoulder.position) # shoulder position
        punkt_2 = copy.copy(self.subject.position)  # subject position
        punkt_3 = copy.copy(self.tcp.position)      # tcp position
        if punkt_1 is not None and punkt_2 is not None and punkt_3 is not None:
            initial_tcp_y  = punkt_1.y +0.3
            initial_tcp_z  = punkt_1.z
            subject_y   = punkt_2.y
            subject_z   = punkt_2.z
            tcp_y       = punkt_3.y
            tcp_z       = punkt_3.z
    
            if initial_tcp_y < 0 or initial_tcp_z < 0 or subject_y < 0 or subject_z < 0 or tcp_y < 0 or tcp_z < 0:
                print 'WARNING: Calculation of error does not work with negative values.'
            else:    
    
                if abs(subject_z - initial_tcp_z) > 0.01: print 'shoulder and subject schould be in one line (same z value)'
                self.value_curve   =  self.curve_data[1, (np.abs(self.curve_data[0] - x)).argmin()]
                self.indice_curve   =  self.curve_data[0, (np.abs(self.curve_data[0] - x)).argmin()]
                self.index_curve   =  np.where(self.curve_data[0] == self.indice_curve)[0][0]
                self.curve_index_normed = self.index_curve / self.curve_data[1].size 
      
      
    def get_elbow_error(self, x):
        self.set_tcp_position(x)
        self.set_value_curve(x)
        return [self.tcp_position_y - self.curve_index_normed  , self.tcp_position_z - self.value_curve]
      
      
      
    def get_shoulder_error(self, x):
        self.set_tcp_position(x)
        self.set_value_curve(x)
        return [self.curve_index_normed  - self.tcp_position_y, self.value_curve - self.tcp_position_z]
        
     
     
     
########################################### L E A R N I N G  - P O L A R ################################# 



    def get_shoulder_error_polar(self,x):
        target_pos = copy.copy(self.tcp.position)
        tcp_pos = copy.copy(self.tcp.position)
        shoulder = copy.copy(self.shoulder.position)
       
        if shoulder is None or target_pos is None:
            return 0.0 
        else:
            self.set_value_curve(x)  
            self.set_tcp_position(x)
            target_pos.z = self.value_curve + shoulder.z
            # Vektor berchnung
            vector_s_tcp = [tcp_pos.x - shoulder.x, tcp_pos.y - shoulder.y, tcp_pos.z - shoulder.z] # PQ  = Q-P
            vector_s_target = [target_pos.x - shoulder.x, target_pos.y - shoulder.y, target_pos.z - shoulder.z]
            dot_product = np.dot(vector_s_tcp, vector_s_target)
            cross_product = np.cross(vector_s_tcp, vector_s_target)
            length_vector_s_tcp = np.linalg.norm(vector_s_tcp)
            length_vector_s_target = np.linalg.norm(vector_s_target)
            sign = cross_product[0] / abs(cross_product[0])
         
            return sign * math.acos(dot_product / (length_vector_s_tcp * length_vector_s_target) )
           
         
    def get_elbow_error_polar(self,x):
        shoulder = copy.copy(self.shoulder.position)
        if shoulder is None:
            return 0.0 
        else:
            self.set_value_curve(x)  
            self.set_tcp_position(x)
            tcp_initial  = shoulder
            tcp_initial.y  = shoulder.y +0.3 # shoulder pos zu initial tcp pos umrechnen
            target_pos = copy.copy(shoulder)
            target_pos.z = self.value_curve + tcp_initial.z
            target_pos.y =  tcp_initial.y - self.curve_index_normed 
            target_pos_vector = self.calc_vector(target_pos)
            target_pos_polar = self.calc_polar(target_pos_vector.x, target_pos_vector.y, target_pos_vector.z)
            return self.tcp.polar_pos[0] - target_pos_polar[0] 


model = nengo.Network()
