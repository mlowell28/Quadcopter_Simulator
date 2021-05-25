from quadcopter_sequential import Quadcopter 
from gui_sequential import GUI
from controller_sequential import LQR_Controller
from path_generator import Path, Waypoint
import signal
import time
import numpy as np
import math
import matplotlib.pyplot as plt


def simulate(use_GUI = True):
    
    print("starting simulation loop")
    
       
    # define path, second argument to waypoint is time, third is optinal yaw 
    # but is only to define ending pose 
    
    waypoint_1 = Waypoint(np.array([0,0,10]), 0,)
    waypoint_2 = Waypoint(np.array([0,5,12]), 40) #20
    waypoint_3 = Waypoint(np.array([14,2,12]), 70) # 60
    waypoint_4 = Waypoint(np.array([3,10,5]), 100) # 80
    waypoint_5 = Waypoint(np.array([0,0,10]), 120, .5)
    
    run_time = 150
    
    # define quadcopter parameters and starting position 
    q1_parameters = {'L':0.3,'r':0.1,'prop_parameters':[10,4.5],'weight':1.2, 'motor_limits':[4000,9000]}
    q1_starting_state = {'position':np.array([0,0,10]), 'linear_rate':np.array([0,0,0]), 'orientation':np.array([0,0,0]), 'angular_rate':np.array([0,0,0])}

    
    # generate interpolated path 
    
    mypath = Path([waypoint_1, waypoint_2, waypoint_3, waypoint_4, waypoint_5]) #Path([waypoint_1, waypoint_2, waypoint_3])
    
    # define LQR cost matrix
    # State space representation: [x y z, x_dot y_dot z_dot, theta phi gamma, omega_1, omega_2, omega_3]
    Q = np.diag([10000,10000,10000,10,10,10,10,10,10000,10,10,10])
    R = np.diag([1,1,1,1])
    
    # create controller and quadcopter
    
    q1_controller = LQR_Controller(mypath, Q, R)
    q1_quadcopter = Quadcopter(q1_parameters, q1_starting_state, q1_controller)
    
    #GUI expects list of quadcopters as input 
    
    if use_GUI == True:
        print("starting gui")
        mygui = GUI([q1_quadcopter])
        
    
    # set step size in seconds
    dt = 1/60
    # set starting time
    t = 0
    
    # compute approximate number of iterations to reach target
    
    loop_count = 0
    
    MAX_LOOP = int(run_time/dt)
    #MAX_LOOP = 4000
    iteration_count = 0
    last_time = time.time()
    
    run = True
    print('starting loop')
    
    state_list = np.zeros((13,MAX_LOOP))
    motor_speed_list = np.zeros((4,MAX_LOOP))
    error_list = np.zeros((4,MAX_LOOP))
    
    while(loop_count < MAX_LOOP):
        [state, t_now] = q1_quadcopter.update(dt)
        
        state_list[0:12, loop_count] = state
        state_list[12, loop_count] = t_now
        
        [m1, m2, m3, m4] = q1_quadcopter.get_motor_speeds()
        
        motor_speed_list[:,loop_count] = [m1, m2, m3, m4]
        
        [x,y,z], yaw = mypath.target_position(t_now)
        
        
        error_list[0, loop_count] = abs(x - state[0])
        error_list[1, loop_count] = abs(y - state[1])
        error_list[2, loop_count] = abs(z - state[2])
        error_list[3, loop_count] = abs(yaw - state[8])
        print("time " + str(t_now))
        print("x error " + str(error_list[0,loop_count]))
        print("y error " + str(error_list[1,loop_count]))
        print("z error " + str(error_list[2,loop_count]))
        print("yaw error " + str(error_list[3,loop_count]))
                
        t += dt
        iteration_count += 1
        loop_count +=1
        
        if use_GUI == True:
            if iteration_count == 60:
                run = mygui.update()
                iteration_count = 0 
        
        print("iteration " + str(loop_count))
    
    # plot positions
    plt.figure()
    plt.plot(state_list[12,:], state_list[0,:])
    plt.title("x position")
    plt.xlabel('s')
    plt.ylabel('m')
    plt.figure()
    plt.plot(state_list[12,:], state_list[1,:])
    plt.title("y position")
    plt.xlabel('s')
    plt.ylabel('m')
    plt.figure()
    plt.plot(state_list[12,:], state_list[2,:])
    plt.title("z position")
    plt.xlabel('s')
    plt.ylabel('m')
    
    # plot yaw
    plt.figure()
    plt.plot(state_list[12,:], state_list[8,:])
    plt.title("yaw")
    plt.xlabel('s')
    plt.ylabel('radians')
    
    #plot positional errors
    plt.figure()
    plt.plot(state_list[12,:], error_list[0,:], label = 'x error')
    plt.plot(state_list[12,:], error_list[1,:], label = 'y error')
    plt.plot(state_list[12,:], error_list[2,:], label = 'z error')
    plt.title("positional errors")
    plt.xlabel('s')
    plt.ylabel('m')
    plt.legend()
    
    # plot yaw error
    plt.figure()
    plt.plot(state_list[12,:], error_list[3,:], label = 'yaw')
    plt.title("yaw error")
    plt.xlabel('s')
    plt.ylabel('radians')
    plt.legend()
    
    #plot motor speeds 
    plt.figure()
    plt.plot(state_list[12,:], motor_speed_list[0,:], label = 'm1')
    plt.plot(state_list[12,:], motor_speed_list[1,:], label = 'm2')
    plt.plot(state_list[12,:], motor_speed_list[2,:], label = 'm3')
    plt.plot(state_list[12,:], motor_speed_list[3,:], label = 'm4')
    plt.title("motor speeds")
    plt.xlabel('s')
    plt.ylabel('RPM')
    plt.legend()
    

if __name__ == "__main__":
    
    simulate(False)
  
    
    

