import numpy as np
import math
import random
import quadcopter_sequential
from scipy import interpolate




class LQR_Controller():
    
    def __init__(self, path, quadcopter_params, controller_params): 
        
        # F = m*a, feed-forward thrust to offset gravity = F/4, 
        
        self.path = path 
        self.simulated_quad = quadcopter()
        
        self.control_point_spacing = controller_params['control_point_spacing']
        self.control_point_horizon = controller_params['control_point_horizon']
        self.dt = controller_params['system_tick']
        
        self.u_1_control_point = np.zeros(self.control_point_horizon)
        self.u_2_control_point = np.zeros(self.control_point_horizon)
        self.u_3_control_point = np.zeros(self.control_point_horizon)
        self.u_4_control_point = np.zeros(self.control_point_horizon)
        
        self.t_span = np.linspace(0, control_point_spacing*dt*control_point_horizon, control_point_horizon)
        
        self.u1 = None
        self.u2 = None
        self.u3 = None
        self.u4 = None
        
        x_0_state = [0,0,0,0,0,0,0,0,0,0,0,0,]
        u_0_state = [0,0,0,0]
        
        # compute linearized model with constant feed-forward term to counter gravity 
        
        [self.A, self.B] = compute_linearization(self, x_0_state, u_0_state)
        
        #compute LQR gain 
        
        # self.MOTOR_LIMITS = params['Motor_limits']
        # self.TILT_LIMITS = [(params['Tilt_limits'][0]/180.0)*3.14,(params['Tilt_limits'][1]/180.0)*3.14]
        # self.YAW_CONTROL_LIMITS = params['Yaw_Control_Limits']
        
        # self.Z_LIMITS = [self.MOTOR_LIMITS[0]+params['Z_XY_offset'],self.MOTOR_LIMITS[1]-params['Z_XY_offset']]
        
    def compute_linearization(self, state_0, input_0, perturbation_value = .01):
        
        jacobian = np.zeros((16,16))
        f_0 = simulated_quad.state_dot_input(state_0, input_0)
        
        for i in range(12):
            
            perturbation_array = np.zeros(12+4)
            perturbation_array[i] = perturbation_value
        
            f_i_plus = simulated_quad.state_dot_input(state_0 + perturbation_array[0:4], input_0 + perturbatino_array[4:12])
            f_i_minus = simulated_quad.state_dot_input(state_0 - perturbation_array[0:4], input_0 - perturbatino_array[4:12])
            
            df_i = np.transpose((f_i_plus - f_0 + f_i_0 - f_i_minus)/(2*perturbation_value)) 
            
            jacobian[:, i] = df_i
            
        A = jacobian[:, :12]
        B = jacobian[:,12:]

        return A, B

    def connect_motors(self, m1, m2, m3, m4):
        self.m1 = m1
        self.m2 = m2
        self.m3 = m3
        self.m4 = m4
        
    def set_motor_limits(self, motor_limit):
        self.motor_limit = motor_limit     
        
        #quadcopter state is [x y z, x_dot y_dot z_dot, theta, phi, gamma, omega_1, omega_2, omega_3]
        # yaw is gamma 
        
    def update(self, t, state):
        
        [x_t, y_t, z_t, yaw] = path(t)
        
        target_state = [x_t, y_t, z_t, 0, 0, 0, 0, 0, yaw, 0, 0, 0]
        
        [u1, u2, u3, u4] = [self.K*(state - target_state)]
        
        self.m1.set_speed(u1)
        self.m2.set_speed(u2)
        self.m3.set_speed(u3)
        self.m4.set_speed(u4)

    def get_target_path(self):
        return self.path
    
    def set_target_path(self, path):
        self.path = path
        
        
        

class Fixed_Control_Executor():
  
    def __init__(self, control_sequence, motor_limits):
        
        # control sequence is a list of 4 functions which are time dependent 
        
        self.control_sequence = control_sequence
        self.motor_limits = motor_limits
    
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
        

class MPC_Controller():
    
    def __init__(self, path, quadcopter_params, controller_params): 
        
        # F = m*a, feed-forward thrust to offset gravity = F/4, 
        
        self.path = path 
        
        # self.MOTOR_LIMITS = params['Motor_limits']
        # self.TILT_LIMITS = [(params['Tilt_limits'][0]/180.0)*3.14,(params['Tilt_limits'][1]/180.0)*3.14]
        # self.YAW_CONTROL_LIMITS = params['Yaw_Control_Limits']
        
        # self.Z_LIMITS = [self.MOTOR_LIMITS[0]+params['Z_XY_offset'],self.MOTOR_LIMITS[1]-params['Z_XY_offset']]
        
        # self.LINEAR_P = params['Linear_PID']['P']
        # self.LINEAR_I = params['Linear_PID']['I']
        # self.LINEAR_D = params['Linear_PID']['D']
        
        # self.LINEAR_TO_ANGULAR_SCALER = params['Linear_To_Angular_Scaler']
        
        # self.YAW_RATE_SCALER = params['Yaw_Rate_Scaler']
        # self.ANGULAR_P = params['Angular_PID']['P']
        # self.ANGULAR_I = params['Angular_PID']['I']
        # self.ANGULAR_D = params['Angular_PID']['D']
        
        # self.xi_term = 0
        # self.yi_term = 0
        # self.zi_term = 0
        # self.thetai_term = 0
        # self.phii_term = 0
        # self.gammai_term = 0
        # self.thread_object = None
        
        # self.target = [0,0,0]
        # self.yaw_target = 0.0
        # self.run = True
        

    def connect_motors(self, m1, m2, m3, m4):
        self.m1 = m1
        self.m2 = m2
        self.m3 = m3
        self.m4 = m4
        
    def set_motor_limits(self, motor_limit):
        self.motor_limit = motor_limit
        
    def update(self, t, state):
        
        if self.ticks == self.control_point_spacing: 
            
            # initialize control points spaced into the future using old points
            # as starting location for search and take last control point
            # to equal the prior last control point
            
            for i in range(len(u_1_control_points)-1):
                u_1_control_points[i] = u_1_control_points[i+1]
                u_2_control_points[i] = u_2_control_points[i+1]
                u_3_control_points[i] = u_3_control_points[i+1]
                u_4_control_points[i] = u_4_control_points[i+1]
                
            # generate splines interpolating the control points 
            
            
                
            
                
            
                
                
            
        else:
            
            
            u_1_control_points
        
        10self.update_count 
        
        u_1_control_points =[]
        u_2_control_points = []
        u_3_control_points = []
        u_4_control_points = []
        
        
        
        
        
        self.m1.set_speed(10000)
        self.m2.set_speed(10000)
        self.m3.set_speed(10000)
        self.m4.set_speed(10000)
        
    def get_target_path(self):
        return self.path


# class LQR_Controller():
    
#     def __init__(self, path, params = None): 
        
#         self.path = path 
        
#         # self.MOTOR_LIMITS = params['Motor_limits']
#         # self.TILT_LIMITS = [(params['Tilt_limits'][0]/180.0)*3.14,(params['Tilt_limits'][1]/180.0)*3.14]
#         # self.YAW_CONTROL_LIMITS = params['Yaw_Control_Limits']
        
#         # self.Z_LIMITS = [self.MOTOR_LIMITS[0]+params['Z_XY_offset'],self.MOTOR_LIMITS[1]-params['Z_XY_offset']]
        
#         # self.LINEAR_P = params['Linear_PID']['P']
#         # self.LINEAR_I = params['Linear_PID']['I']
#         # self.LINEAR_D = params['Linear_PID']['D']
        
#         # self.LINEAR_TO_ANGULAR_SCALER = params['Linear_To_Angular_Scaler']
        
#         # self.YAW_RATE_SCALER = params['Yaw_Rate_Scaler']
#         # self.ANGULAR_P = params['Angular_PID']['P']
#         # self.ANGULAR_I = params['Angular_PID']['I']
#         # self.ANGULAR_D = params['Angular_PID']['D']
        
#         # self.xi_term = 0
#         # self.yi_term = 0
#         # self.zi_term = 0
#         # self.thetai_term = 0
#         # self.phii_term = 0
#         # self.gammai_term = 0
#         # self.thread_object = None
        
#         # self.target = [0,0,0]
#         # self.yaw_target = 0.0
#         # self.run = True
        

#     def connect_motors(self, m1, m2, m3, m4):
#         self.m1 = m1
#         self.m2 = m2
#         self.m3 = m3
#         self.m4 = m4
        
#     def set_motor_limits(self, motor_limit):
#         self.motor_limit = motor_limit
        
#     def update(self, t, state):
#         target = self.path.target_position(t)
        
#     def get_target_path(self):
#         return self.path



# class PID_Controller():
#     def __init__(self, path, params):
        
#         self.path = path
        
#         self.MOTOR_LIMITS = params['Motor_limits']
#         self.TILT_LIMITS = [(params['Tilt_limits'][0]/180.0)*3.14,(params['Tilt_limits'][1]/180.0)*3.14]
#         self.YAW_CONTROL_LIMITS = params['Yaw_Control_Limits']
#         self.Z_LIMITS = [self.MOTOR_LIMITS[0]+params['Z_XY_offset'],self.MOTOR_LIMITS[1]-params['Z_XY_offset']]
#         self.LINEAR_P = params['Linear_PID']['P']
#         self.LINEAR_I = params['Linear_PID']['I']
#         self.LINEAR_D = params['Linear_PID']['D']
#         self.LINEAR_TO_ANGULAR_SCALER = params['Linear_To_Angular_Scaler']
#         self.YAW_RATE_SCALER = params['Yaw_Rate_Scaler']
#         self.ANGULAR_P = params['Angular_PID']['P']
#         self.ANGULAR_I = params['Angular_PID']['I']
#         self.ANGULAR_D = params['Angular_PID']['D']
#         self.xi_term = 0
#         self.yi_term = 0
#         self.zi_term = 0
#         self.thetai_term = 0
#         self.phii_term = 0
#         self.gammai_term = 0
#         self.thread_object = None
#         self.target = [0,0,0]
#         self.yaw_target = 0.0
#         self.run = True
    
#     def connect_motors(self, m1, m2, m3, m4):
#         self.m1 = m1
#         self.m2 = m2
#         self.m3 = m3
#         self.m4 = m4

#     def wrap_angle(self,val):
#         return( ( val + np.pi) % (2 * np.pi ) - np.pi )

#     def update(self, t, state):
        
#         [dest_x,dest_y,dest_z, dest_yaw] = self.path(t)
        
#         [x,y,z,x_dot,y_dot,z_dot,theta,phi,gamma,theta_dot,phi_dot,gamma_dot] = self.state
        
#         x_error = dest_x-x
#         y_error = dest_y-y
#         z_error = dest_z-z
        
#         self.xi_term += self.LINEAR_I[0]*x_error
#         self.yi_term += self.LINEAR_I[1]*y_error
#         self.zi_term += self.LINEAR_I[2]*z_error
        
#         dest_x_dot = self.LINEAR_P[0]*(x_error) + self.LINEAR_D[0]*(-x_dot) + self.xi_term
#         dest_y_dot = self.LINEAR_P[1]*(y_error) + self.LINEAR_D[1]*(-y_dot) + self.yi_term
#         dest_z_dot = self.LINEAR_P[2]*(z_error) + self.LINEAR_D[2]*(-z_dot) + self.zi_term
        
#         throttle = np.clip(dest_z_dot,self.Z_LIMITS[0],self.Z_LIMITS[1])
        
#         dest_theta = self.LINEAR_TO_ANGULAR_SCALER[0]*(dest_x_dot*math.sin(gamma)-dest_y_dot*math.cos(gamma))
#         dest_phi = self.LINEAR_TO_ANGULAR_SCALER[1]*(dest_x_dot*math.cos(gamma)+dest_y_dot*math.sin(gamma))
#         dest_gamma = self.dest_yaw
        
#         dest_theta,dest_phi = np.clip(dest_theta,self.TILT_LIMITS[0],self.TILT_LIMITS[1]),np.clip(dest_phi,self.TILT_LIMITS[0],self.TILT_LIMITS[1])
        
#         theta_error = dest_theta-theta
#         phi_error = dest_phi-phi
#         gamma_dot_error = (self.YAW_RATE_SCALER*self.wrap_angle(dest_gamma-gamma)) - gamma_dot
        
#         self.thetai_term += self.ANGULAR_I[0]*theta_error
#         self.phii_term += self.ANGULAR_I[1]*phi_error
#         self.gammai_term += self.ANGULAR_I[2]*gamma_dot_error
        
#         x_val = self.ANGULAR_P[0]*(theta_error) + self.ANGULAR_D[0]*(-theta_dot) + self.thetai_term
#         y_val = self.ANGULAR_P[1]*(phi_error) + self.ANGULAR_D[1]*(-phi_dot) + self.phii_term
#         z_val = self.ANGULAR_P[2]*(gamma_dot_error) + self.gammai_term
#         z_val = np.clip(z_val,self.YAW_CONTROL_LIMITS[0],self.YAW_CONTROL_LIMITS[1])
        
#         m1_speed = throttle + x_val + z_val
#         m2_speed = throttle + y_val - z_val
#         m3_speed = throttle - x_val + z_val
#         m4_speed = throttle - y_val - z_val
        
#         [m1_speed, m2_speed, m3_speed, m4_speed] = np.clip([m1_speed, m2_speed ,m3_speed ,m4_speed],self.MOTOR_LIMITS[0],self.MOTOR_LIMITS[1])

#         self.m1.set_speed(m1_speed)
#         self.m2.set_speed(m2_speed)
#         self.m3.set_speed(m3_speed)
#         self.m4.set_speed(m4_speed)

#     def update_path(self, path):
#         self.path = path
    
