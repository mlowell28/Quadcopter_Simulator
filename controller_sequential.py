import numpy as np
import math
import random
import quadcopter_sequential
from scipy import interpolate
import control

class LQR_Controller():
    
    def __init__(self, path, Q, R): 
        
        # add path object to controller
        
        self.path = path 
        
        # set LQR Q and R cost matrix
        
        self.Q = Q
        self.R = R
        
        self.feedback_matrix_list = []
        
    def add_quadcopter(self, quadcopter):
        
        # attach quadcopter to controller
        
        self.quad = quadcopter
        self.connect_motors(quadcopter.m1, quadcopter.m2, quadcopter.m3, quadcopter.m4)   
        
        # set motor speed limits
        
        self.motor_limits = quadcopter.parameters['motor_limits']
        
        # build feedback matrix
        
        self.compute_feedback_matrix()

    def compute_feedback_matrix(self, x_0_state=None, u_0_state=None):
          
        # want equilbrium_point at hover, so motors must offset gravity while
        # producing no net torque, so computing equilbrium speed for motors 

        F_g = self.quad.g*self.quad.parameters['weight'] 
        F_m = 1/4*F_g
        
        m_s = self.m1.get_speed(F_m)
        
        # using hover based motor speed offset if no state is given 
        
        if u_0_state == None:
            self.u_0_state = [m_s, m_s, m_s, m_s]
        
       # translation invariance holds but yaw invariance does not,
       # thus we compute feedback for a spacing of yaw values and select matrix
       # based upon which yaw value is closest to the target yaw value
        
        if x_0_state == None:
            yaw_angles = np.linspace(-math.pi,math.pi, 100)
            
            for yaw_angle in yaw_angles:
                self.x_0_state = [0,0,0,0,0,0,0,0,yaw_angle,0,0,0]
        
            # use finite difference approximation to compute jacobian of dynamics
            # around hover state
        
                A, B = self.compute_linearization(self.x_0_state, self.u_0_state)
                
                C = control.ctrb(A, B)
                print("rank of controllability matrix at state " + str(self.x_0_state) + " is "  + str(np.linalg.matrix_rank(C)))
                
                #compute LQR gain and add to dictionary 
                
                K, S, E = control.lqr(A, B, self.Q, self.R)
                
                self.feedback_matrix_list.append([yaw_angle, K])
                
            return self.feedback_matrix_list

        else:
        
        # use finite difference approximation to compute jacobian of dynamics
        # around hover state
    
            A, B = self.compute_linearization(x_0_state, u_0_state)
        
            C = control.ctrb(self.A, self.B)
    
            print("rank of controllability matrix at state " + str(x_0_state) + " is "  + str(np.linalg.matrix_rank(C)))
        
        #compute LQR gain
            
            K, S, E = control.lqr(A, B, self.Q, self.R)
            
            return K
            
        
    
    # compte approximate jacobian using average finite difference with given
    # perturbation values 
        
    def compute_linearization(self, state_0, input_0, perturbation_value = .000001):
        
        jacobian = np.zeros((12,16))
        for i in range(16):
            
            perturbation_array = np.zeros(12+4)
            perturbation_array[i] = perturbation_value
        
            f_i_plus = self.quad.state_dot(0, state_0 + perturbation_array[0:12], input_0 + perturbation_array[12:16])  
            f_i_minus = self.quad.state_dot(0, state_0 - perturbation_array[0:12], input_0 - perturbation_array[12:16])
            
            df_i = np.transpose((f_i_plus - f_i_minus)/(2*perturbation_value)) 
            
            jacobian[:, i] = df_i
            
        # split jacobian into state linearization and control linearization
        
        A = jacobian[:, :12]
        B = jacobian[:,12:]

        return A, B

    # attach motor objects to controller
    
    def connect_motors(self, m1, m2, m3, m4):
        self.m1 = m1
        self.m2 = m2
        self.m3 = m3
        self.m4 = m4
    
    # set motor limits in the form of [low, high], update will clip speeds to be
    # within values
    
    def set_motor_limits(self, motor_limit):
        self.motor_limit = motor_limit     
        

    # quadcopter state is [x y z, x_dot y_dot z_dot, theta, phi, gamma, omega_1, omega_2, omega_3]
    # yaw is gamma 
     
    def update(self, t, state):
        
        target_state = self.path.target_position(t)

        # we want the quadcopter to track the target equilibrium position,
        # by varying it we can force the quadcopter to track a trajectory
        # using the error between current state and target state as our input, we sekcan
        # force the quadcopter to seek an error of 0.
        
        # compute yaw angle taking the minimum between the target and position using wrap around
        
        yaw_error = state[8] - target_state[8]
        
        # wrap around yaw error so shortest radian is always used.
        
        if yaw_error >= math.pi:
            yaw_error = -1*(2*math.pi - yaw_error)  
        elif yaw_error < -1*math.pi:
            yaw_error = (2*math.pi + yaw_error) 
        
        # find closest feedback matrix to target yaw
        best_yaw_dist = 10
        closest_yaw = 0
        for i in range(len(self.feedback_matrix_list)):
            yaw_dist = abs(self.wrap_angle(self.feedback_matrix_list[i][0] - target_state[8]))
            if yaw_dist < best_yaw_dist:
                best_yaw_dist = yaw_dist
                closest_yaw = self.feedback_matrix_list[i][0]
                K = self.feedback_matrix_list[i][1]
                
        error = state - target_state 
        error[8] = self.wrap_angle(error[8])
        #error[8] = state[8] - closest_yaw
        
        [u1, u2, u3, u4] = -1*np.matmul(K,error) + self.u_0_state
        [u1, u2, u3, u4] = np.clip([u1, u2, u3, u4], self.motor_limits[0], self.motor_limits[1])
        
        self.m1.set_speed(u1)
        self.m2.set_speed(u2)
        self.m3.set_speed(u3)
        self.m4.set_speed(u4)
        
    def get_feedback(self, t, state):
        
        target_state = np.zeros(12)
        
        target_state = self.path.target_position(t)

        # we want the quadcopter to track the target equilibrium position,
        # by varying it we can force the quadcopter to track a trajectory
        
        # using the error between current state and target state as our input, we sekcan
        # force the quadcopter to seek an error of 0.
        
        # compute yaw angle taking the minimum between the target and position using wrap around
        
        yaw_error = state[8] - target_state[8]
        
        # wrap around yaw error so shortest radian is always used.
        
        if yaw_error >= math.pi:
            yaw_error = -1*(2*math.pi - yaw_error)  
        elif yaw_error < -1*math.pi:
            yaw_error = (2*math.pi + yaw_error) 
            
        error = state - target_state 
        error[8] = yaw_error
        
        [u1, u2, u3, u4] = -1*np.matmul(self.K,error) + self.u_0_state
        
        return [u1, u2, u3, u4]
        

    # return the path object the controller is using
    
    def get_target_path(self):
        return self.path
    
    # update the path object the controller is using
    
    def set_target_path(self, path):
        self.path = path
        
    def wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )

        
     
    # create a controller which executes a fixed control sequence where
    # control sequence is an array of 4 functions which take time as input

class Fixed_Control_Executor():
  
    def __init__(self, control_sequence, motor_limits):
        
        # control sequence is a list of 4 functions which are time dependent 
        
        self.control_sequence = control_sequence
        self.motor_limits = motor_limits 
        
    def add_quadcopter(self, quadcopter):
            
        # attach quadcopter to controller
        
        self.quad = quadcopter
        self.connect_motors(quadcopter.m1, quadcopter.m2, quadcopter.m3, quadcopter.m4)   
        
        # set motor speed limits
        
        self.motor_limits = quadcopter.parameters['motor_limits']
        

    def connect_motors(self, m1, m2, m3, m4):
        self.m1 = m1
        self.m2 = m2
        self.m3 = m3
        self.m4 = m4
        
    def set_motor_limits(self, motor_limit):
        self.motor_limit = motor_limit
        
    def update(self, t, state):
        
        u1 = np.clip(self.control_sequence[0](t), self.motor_limit)
        u2 = np.clip(self.control_sequence[1](t), self.motor_limit)
        u3 = np.clip(self.control_sequence[2](t), self.motor_limit)
        u4 = np.clip(self.control_sequence[3](t), self.motor_limit)
        
        self.m1.set_speed(u1)
        self.m2.set_speed(u2)
        self.m3.set_speed(u3)
        self.m4.set_speed(u4)

        
        
# class MPC_Controller():
    
#     def __init__(self, path, parameters): 
        
#         # add path object to controller
        
#         self.path = path 
#         self.control_points = []
#         self.control_point_spacing = parameters['control_point_spacing']
#         self.horizion_length = parameters['horizion_length']

        
#     def add_quadcopter(self, quadcopter):
        
#         # attach quadcopter to controller
        
#         self.quad = quadcopter
#         self.connect_motors(quadcopter.m1, quadcopter.m2, quadcopter.m3, quadcopter.m4)   
        
#         # set motor speed limits
        
#         self.motor_limits = quadcopter.parameters['motor_limits'] 
        
#         F_g = self.quad.g*self.quad.parameters['weight'] 
#         F_m = 1/4*F_g
        
#         m_s = self.m1.get_speed(F_m)
        
#         # using hover based motor speed offset
        
#         self.input_offset = [m_s, m_s, m_s, m_s]
        

#     # attach motor objects to controller
    
#     def connect_motors(self, m1, m2, m3, m4):
#         self.m1 = m1
#         self.m2 = m2
#         self.m3 = m3
#         self.m4 = m4
    
#     # set motor limits in the form of [low, high], update will clip speeds to be
#     # within values
    
#     def set_motor_limits(self, motor_limit):
#         self.motor_limit = motor_limit     
       
#     # u1, u2, u3, u4 are continous functions of time
#     def compute_MPC_cost(self, state_0, t, u1, u2, u3, u4, path, state_weight_matrix = np.ones((12,12)), control_weight_matrix = np.ones((4,4)), dt =.1, t_len = 1):
        
#         # horizion length
              
#         state_cost = 0
#         control_cost = 0
        
#         state_weight_matrix = []
#         control_weight_matrix = []
        
#         while t < t_len:  
#             state_dot = self.quad.state_dot(t, state_0, u1(t), u2(t), u3(t), u4(t))
#             new_state = state_dot*dt + old_state   
#             state_cost += np.matmul((np.transpose(new_state - path(t)), np.matmul(state_weight_matrix, new_state - path(t))))
#             control_cost += np.matmul(np.matmul(np.transpose(np.array([u1, u2, u3, u4]), state_weight_matrix), np.array([u1,u2,u3,u4])))*dt
#             t += dt
    
#         return state_cost + control_cost 
    
#     def compute_control_derivative(self, state_0):
        
#         np.horizion
        
#         # compute base cost
        
#         # computer derivative WRT to control points
        
#         control_jacobian = np.zeros()
        
#         for control_index in range(self.horizion_length*4):
#             control_point[control_index] += perturbation_value 
#             u1, u2, u3, u4 = generate_control_signal(control_points)
            
#             cost_pertb = compute_MPC_cost(self, control_signal, state_0)
            
#             dC_du = (cost_pertb - cost)/perturbation_value
            
#             cost_jacobian.append(dC_du)
        
#         return cost_jacobian
    
    
        
#         # apply gradient descent
        
#         for control_index in range(self.horizion_length*4):
#             control_point[control_index ] += learning_rate*cost_jacobian[control_index]
        
    
    
#     def generate_control_signal(self, control_points):
        
#         u1 = scipy.interpolate()
#         u2 = scipy.interpolate
#         u3 = scipy.interpolate
#         u4 = scipy.interpolate 
        
        
    
#     def update(self, t, state):
        
#         u1_control_points = np.zeros((2))
#         u2_control_points = np.zeros((2))
#         u3_control_points = np.zeros((2))
#         u4_control_points = np.zeros((2))
        
#         u1_control_points[:3] = self.prior_u1_control_points[1:]
#         u1_control_points[3] = u1_control_points[4]
        
        
        
#         self.u1_last_control_points
#         self.u2_last_control_points
#         self.u3_last_control_points 
        
#         control_spacing = .5
        
#         # compute base cost for control
        
#         u1
#         u2
#         u3
#         u4
        
#         t_p = [i:i*control_spacing ]
        
#         while(done == False):
            
#             t_p = list(range(2))*control_spacing
            
#             cost = compute_MPC_cost(state_0, t, u1, u2, u3, u4)
            
#             control_point_count = horizion*4
#             control_points = np.zeros(control_point_count)
            
#             for j in range(5):
#                 for i in range(4):
                    
#                     peturbation_array = np.zeros(control_points_count)
#                     perturbation_matrix[i*j] = peturbation_value
#                     perturbed_control_points = control_points + perturbation_matrix
                    
#                     u1_control_points = []
#                     u2_control_points = []
#                     u3_control_points =[]
#                     u4_control_points = []
                    
#                     # generate control interpolation 
                    
#                     u1_p = interpolate.interp1d(t_p,x_p, path_type))
                    
#                     # compute cost iwth perturbed values
                    
#                     cost = compute_MPC_cost(state_0, t, u1, u2, u3, u4) 
#                     cost_matrix[j*i](cost)
                    
            
#             jacobian = (cost - cost_perturbed)/dt   
#             control_points = control_points - alpha*control_point_jacobian 
            
            
            
#             while not_converged:
                
                
#                 for control_point in range(10):
#                     for motor in range(4):
                        
#                         f_u1 = 
#                         f_u2 = 
#                         f_u3 = 
#                         f_u4 = 
                        
                    
                    
                    
                    
            
                
                
                
        
        
#         if self.update_count = 50:
            
#             # take linear feedback as starting point 
            
            
            
#             t_p = linspace(1, 10, )
             
#             self.f_u1 = interpolate.interp1d(t_p,x_p, path_type)
#             self.f_u2 
#             self.f_u3
#             self.f_u4
            
#             error = lambda  
            
        
        
        
#         [u1, u2, u3, u4] = np.clip([self.u1(t), self.u2(t), self.u3(t), self.u4(t)], self.motor_limits[0], self.motor_limits[1])
        
#         self.m1.set_speed(u1)
#         self.m2.set_speed(u2)
#         self.m3.set_speed(u3)
#         self.m4.set_speed(u4)
            
        
#     # return the path object the controller is using
    
#     def get_target_path(self):
#         return self.path
    
#     # update the path object the controller is using
    
#     def set_target_path(self, path):
#         self.path = path

        
