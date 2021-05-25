import numpy as np
import math
from scipy.integrate import solve_ivp


# Propeller class which computs thrust from speed, also computes speed needed
# for a given thrust. 

class Propeller():
    
    # define propeller parameters  
    def __init__(self, prop_dia, prop_pitch, thrust_unit='N'):
        self.dia = prop_dia
        self.pitch = prop_pitch
        self.thrust_unit = thrust_unit
        self.speed = 0 #RPM
        self.thrust = 0
        self.thrust_unit = thrust_unit
        
    # set speed and resulting thrust, note that thrust is the square of speed
    def set_speed(self,speed):
        self.speed = speed
        # From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
        self.thrust = 4.392e-8 * self.speed * math.pow(self.dia,3.5)/(math.sqrt(self.pitch))
        self.thrust = self.thrust*(4.23e-4 * self.speed * self.pitch)
        if self.thrust_unit == 'Kg':
            self.thrust = self.thrust*0.101972
        return self.thrust
            
    # compute thrust for a given speed without saving to propeller object
    def get_thrust(self,speed = None):
        
        if speed == None:
            return self.thrust
        else:       
            thrust = 4.392e-8 * speed * math.pow(self.dia,3.5)/(math.sqrt(self.pitch))
            thrust = thrust*(4.23e-4 * speed * self.pitch)
            if self.thrust_unit == 'Kg':
                self.thrust = self.thrust*0.101972
            return thrust
        
    # returns current speed, if thrust is given as input computes required speed
    # to produce thrust in given unit
    
    def get_speed(self, thrust= None, unit = 'N'):
        
        if thrust == None:        
            return self.speed
        
        speed = math.sqrt(thrust/(4.392e-8  * math.pow(self.dia,3.5)/(math.sqrt(self.pitch))*(4.23e-4*self.pitch)))
        return speed
                          

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
        self.controller.add_quadcopter(self)

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
        #matrix = np.array([[1,0, -st],[0, cp, ct*sp],[0,-sp, ct*cp]])
        #return np.linalg.inv(matrix)
        
        matrix = np.array([[1, sp*st/ct, cp* st/ct],
                           [0, cp, -sp],
                           [0, sp*1/ct, cp*1/ct]])
        return matrix

    def wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )

    def state_dot(self, t, state = [], control_input = []):
        
        if len(control_input) == 0:
            m1_thrust = self.m1.thrust
            m2_thrust = self.m2.thrust
            m3_thrust = self.m3.thrust
            m4_thrust = self.m4.thrust
            
        else:
            
            m1_thrust = self.m1.get_thrust(control_input[0])
            m2_thrust = self.m2.get_thrust(control_input[1])
            m3_thrust = self.m3.get_thrust(control_input[2])
            m4_thrust = self.m4.get_thrust(control_input[3])
            
        if len(state) == 0:
            
            state = self.state
            
        
        state_dot = np.zeros(12)
        # The velocities(t+1 x_dots equal the t x_dots)
        state_dot[0] = state[3]
        state_dot[1] = state[4]
        state_dot[2] = state[5]
        # 
        x_dotdot = (np.array([0,0,-self.g*self.parameters['weight']]) + np.dot(self.rotation_matrix(state[6:9]),np.array([0,0,(m1_thrust + m2_thrust + m3_thrust + m4_thrust)])))/self.parameters['weight']
        state_dot[3] = x_dotdot[0]
        state_dot[4] = x_dotdot[1]
        state_dot[5] = x_dotdot[2]

        euler_angles = state[6:9]
        omega = state[9:12]

        # relate euler angle derivative to euler and angular velocity vector
        
        state_dot[6:9] = np.matmul(self.angular_velocity_transformation_matrix(euler_angles), omega)

        # The derivative of the angular velocity vector 
        tau = np.array([self.parameters['L']*(m1_thrust-m3_thrust), self.parameters['L']*(m2_thrust-m4_thrust), self.b*(m1_thrust-m2_thrust+m3_thrust-m4_thrust)])
        omega_dot = np.dot(self.Iinv, (tau - np.cross(omega, np.dot(self.I,omega))))
        
        state_dot[9:12] = omega_dot

        return state_dot

    def update(self, dt):
         
        self.controller.update(self.t, self.state)
        ivp_out = solve_ivp(self.state_dot, [self.t, self.t + dt], self.state)
        self.state = ivp_out.y[:,1]
        print('z value is ' + str(self.state[2]))
        self.state[6:9] = self.wrap_angle(self.state[6:9])
        self.state[2] = max(0,self.state[2])
        self.t += dt 
        
        return [self.state, self.t]

    def set_motor_speeds(self,speeds):
        self.m1.set_speed(speeds[0])
        self.m2.set_speed(speeds[1])
        self.m3.set_speed(speeds[2])
        self.m4.set_speed(speeds[3])
        
    def get_motor_speeds(self):
        return [self.m1.get_speed(), self.m2.get_speed(), self.m3.get_speed(), self.m4.get_speed()]

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



