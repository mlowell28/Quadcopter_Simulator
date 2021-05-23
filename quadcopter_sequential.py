import numpy as np
import math
import scipy.integrate
import time
import datetime
import threading

class Propeller():
    def __init__(self, prop_dia, prop_pitch, thrust_unit='N'):
        self.dia = prop_dia
        self.pitch = prop_pitch
        self.thrust_unit = thrust_unit
        self.speed = 0 #RPM
        self.thrust = 0

    def set_speed(self,speed):
        self.speed = speed
        # From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
        self.thrust = 4.392e-8 * self.speed * math.pow(self.dia,3.5)/(math.sqrt(self.pitch))
        self.thrust = self.thrust*(4.23e-4 * self.speed * self.pitch)
        if self.thrust_unit == 'Kg':
            self.thrust = self.thrust*0.101972

class Quadcopter():
    # State space representation: [x y z, x_dot y_dot z_dot, theta phi gamma, omega_1, omega_2, omega_3]
    # From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
    def __init__(self, parameters, starting_state, controller, gravity=9.81,b=0.0245):
        
        self.t = 0
        
        self.parameters = parameters
        self.g = gravity
        self.b = b
        
        self.state = np.zeros(12)
        self.state[0:3] = starting_state['position']
        self.state[3:6] = starting_state['linear_rate']
        self.state[6:9] = starting_state['orientation']
        self.state[9:12] = starting_state['angular_rate']
        
  
        self.ode =  scipy.integrate.ode(self.state_dot).set_integrator('vode',nsteps=500,method='bdf')
        
        # From Quadrotor Dynamics and Control by Randal Beard
        ixx=((2*self.parameters['weight']*self.parameters['r']**2)/5)+(2*self.parameters['weight']*self.parameters['L']**2)
        iyy=ixx
        izz=((2*self.parameters['weight']*self.parameters['r']**2)/5)+(4*self.parameters['weight']*self.parameters['L']**2)
        
        self.I = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])
        self.Iinv = np.linalg.inv(self.I)
        
        self.m1 = Propeller(self.parameters['prop_parameters'][0],self.parameters['prop_parameters'][1])
        self.m2 = Propeller(self.parameters['prop_parameters'][0],self.parameters['prop_parameters'][1])
        self.m3 = Propeller(self.parameters['prop_parameters'][0],self.parameters['prop_parameters'][1])
        self.m4 = Propeller(self.parameters['prop_parameters'][0],self.parameters['prop_parameters'][1])
        
        self.controller = controller
        self.controller.connect_motors(self.m1, self.m2, self.m3, self.m4)
        self.controller.set_motor_limits(parameters['motor_limits'])

    def rotation_matrix(self,angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        cg = math.cos(angles[2])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        sg = math.sin(angles[2])
        R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
        R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R

    def angular_velocity_transformation_matrix(self, angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        matrix = np.array([[1,0, -st],[0, cp, ct*sp],[0,-sp, ct*cp]])
        return np.linalg.inv(matrix)


    def wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )

    def state_dot(self, t, state):
        
        state_dot = np.zeros(12)
        # The velocities(t+1 x_dots equal the t x_dots)
        state_dot[0] = self.state[3]
        state_dot[1] = self.state[4]
        state_dot[2] = self.state[5]
        # The acceleration
        x_dotdot = np.array([0,0,-self.parameters['weight']*self.g]) + np.dot(self.rotation_matrix(self.state[6:9]),np.array([0,0,(self.m1.thrust + self.m2.thrust + self.m3.thrust + self.m4.thrust)]))/self.parameters['weight']
        state_dot[3] = x_dotdot[0]
        state_dot[4] = x_dotdot[1]
        state_dot[5] = x_dotdot[2]

        euler_angles = self.state[6:9]
        omega = self.state[9:12]

        # relate euler angle derivative to euler and angular velocity vector
        
        state_dot[6:9] = np.matmul(self.angular_velocity_transformation_matrix(euler_angles), omega)

        # The derivative of the angular velocity vector 
        tau = np.array([self.parameters['L']*(self.m1.thrust-self.m3.thrust), self.parameters['L']*(self.m2.thrust-self.m4.thrust), self.b*(self.m1.thrust-self.m2.thrust+self.m3.thrust-self.m4.thrust)])
        omega_dot = np.dot(self.Iinv, (tau - np.cross(omega, np.dot(self.I,omega))))
        
        state_dot[9:12] = omega_dot

        return state_dot

    def update(self, dt):
        
        self.controller.update(self.t, self.state)
        self.ode.set_initial_value(self.state,0)
        self.state = self.ode.integrate(self.ode.t + dt)
        self.state[6:9] = self.wrap_angle(self.state[6:9])
        self.state[2] = max(0,self.state[2])
        self.t = self.t+dt
        
        return [self.state, self.t]

    def set_motor_speeds(self,speeds):
        self.m1.set_speed(speeds[0])
        self.m2.set_speed(speeds[1])
        self.m3.set_speed(speeds[2])
        self.m4.set_speed(speeds[3])

    def get_position(self):
        return self.state[0:3]

    def get_linear_rate(self):
        return self.state[3:6]

    def get_orientation(self):
        return self.state[6:9]

    def get_angular_rate(self):
        return self.state[9:12]

    def get_state(self):
        return self.state

    def set_position(self,position):
        self.state[0:3] = position

    def set_orientation(self,orientation):
        self.state[6:9] = orientation
        
    def set_linear_rate(self, velocity):
        self.state[3:6] = velocity
        
    def set_angular_rate(self, angular_rate):
        self.state[9:12] = angular_rate

    def get_time(self):
        return self.time
    
    def get_target_path(self):
        return self.controller.get_target_path()



