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
    
    waypoint_1 = Waypoint(np.array([0,0,10]), 0, 0)
    waypoint_2 = Waypoint(np.array([0,10,15]), 60) #20
    waypoint_3 = Waypoint(np.array([10,20,5]), 120) # 60
    waypoint_4 = Waypoint(np.array([10,15,10]), 180) # 80
    waypoint_5 = Waypoint(np.array([0,0,10]), 240, .5)
    
    run_time = 270
    
    # define quadcopter parameters and starting position 
    q1_parameters = {'L':0.3,'r':0.1,'prop_parameters':[10,4.5],'weight':1.2, 'motor_limits':[4000,9000]}
    q1_starting_state = {'position':np.array([0,0,10]), 'linear_rate':np.array([0,0,0]), 'orientation':np.array([0,0,0]), 'angular_rate':np.array([0,0,0])}

    
    # generate interpolated path 
    mypath = Path([waypoint_1, waypoint_2, waypoint_3, waypoint_4, waypoint_5], 'quadratic')
    #mypath = Path([waypoint_1, waypoint_2, waypoint_3, waypoint_4, waypoint_5]) #Path([waypoint_1, waypoint_2, waypoint_3])
    
    # define LQR cost matrix
    # State space rpresentation: [x y z, x_dot y_dot z_dot, theta phi gamma, omega_1, omega_2, omega_3]
    Q = np.diag([100,100,100,10,10,10,10,10,10,1,1,1])
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
    
    iteration_count = 0
    
    run = True
    print('starting loop')
    
    state_list = np.zeros((13,MAX_LOOP))
    target_position_list = np.zeros((4,MAX_LOOP))
    motor_speed_list = np.zeros((4,MAX_LOOP))
    error_list = np.zeros((4,MAX_LOOP))
    
    while(loop_count < MAX_LOOP):
        [state, t_now] = q1_quadcopter.update(dt)
        
        state_list[0:12, loop_count] = state
        state_list[12, loop_count] = t_now
        
        
        [m1, m2, m3, m4] = q1_quadcopter.get_motor_speeds()
        
        motor_speed_list[:,loop_count] = [m1, m2, m3, m4]
        
        [x,y,z], yaw = mypath.target_position(t_now)
        
        target_position_list[:, loop_count] = [x, y, z, yaw]
        
        error_list[0, loop_count] = abs(x - state[0])
        error_list[1, loop_count] = abs(y - state[1])
        error_list[2, loop_count] = abs(z - state[2])
        
        yaw_error = state[8] - yaw
        
        if yaw_error > math.pi:
            yaw_error = -1*(2*math.pi - yaw_error)
        elif yaw_error < -1*math.pi:
            yaw_error = (2*math.pi + yaw_error)
        
        
        error_list[3, loop_count] = abs(yaw_error)
        
        print("time " + str(t_now))
        print("x error " + str(error_list[0,loop_count]))
        print("y error " + str(error_list[1,loop_count]))
        print("z error " + str(error_list[2,loop_count]))
        print("yaw error " + str(error_list[3,loop_count]))
        print("target yaw " + str(yaw) + " current_yaw " + str(state[8])) 
        
                
        t += dt
        iteration_count += 1
        loop_count +=1
        
        if use_GUI == True:
            if iteration_count == 30:
                run = mygui.update()
                iteration_count = 0 
        
        print("iteration " + str(loop_count))
        
    
    # plot positions
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[0,:loop_count], label='true')
    plt.title("x position")
    plt.plot(state_list[12,:loop_count], target_position_list[0,:loop_count], label='target')
    plt.legend()
    plt.xlabel('s')
    plt.ylabel('m')
    
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[1,:loop_count], label='true')
    plt.plot(state_list[12,:loop_count], target_position_list[1,:loop_count], label='target')
    plt.legend()
    plt.title("y position")
    plt.xlabel('s')
    plt.ylabel('m')
    
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[2,:loop_count], label='true')
    plt.plot(state_list[12,:loop_count], target_position_list[2,:loop_count], label='target')
    plt.legend()
    plt.title("z position")
    plt.xlabel('s')
    plt.ylabel('m')
    
    # plot yaw
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[8,:loop_count], label='true')
    plt.plot(state_list[12,:loop_count], target_position_list[3,:loop_count],label='target')
    plt.legend()
    plt.title("yaw")
    plt.xlabel('s')
    plt.ylabel('radians')
    
     
    # plot other two angles angles
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[6,:loop_count], label='true')
    plt.title("theta ")
    #plt.plot(state_list[12,:loop_count], target_position_list[0,:loop_count], label='target')
    plt.legend()
    plt.xlabel('s')
    plt.ylabel('radians')
    
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[7,:loop_count], label='true')
   # plt.plot(state_list[12,:loop_count], target_position_list[1,:loop_count], label='target')
    plt.legend()
    plt.title("phi")
    plt.xlabel('s')
    plt.ylabel('radians')
    
    
    # plot yaw
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[8,:loop_count], label='true')
    #plt.plot(state_list[12,:loop_count], target_position_list[3,:loop_count],label='target')
    plt.legend()
    plt.title("yaw")
    plt.xlabel('s')
    plt.ylabel('radians')
    
    # # plot target positions
    # plt.figure()
    # plt.plot(state_list[12,:loop_count], target_position_list[0,:loop_count])
    # plt.title("target x")
    # plt.xlabel('s')
    # plt.ylabel('radians')
    # # plot target yaw
    # plt.figure()
    # plt.plot(state_list[12,:loop_count], target_position_list[1,:loop_count])
    # plt.title("target y")
    # plt.xlabel('s')
    # plt.ylabel('radians')
    # # plot target yaw
    # plt.figure()
    # plt.plot(state_list[12,:loop_count], target_position_list[2,:loop_count])
    # plt.title("target z")
    # plt.xlabel('s')
    # plt.ylabel('radians')
    
    
    # # plot target yaw
    # plt.figure()
    # plt.plot(state_list[12,:loop_count], target_position_list[3,:loop_count])
    # plt.title("target yaw")
    # plt.xlabel('s')
    # plt.ylabel('radians')
    
    
    #plot positional errors
    plt.figure()
    plt.plot(state_list[12,:loop_count], error_list[0,:loop_count], label = 'x error')
    plt.plot(state_list[12,:loop_count], error_list[1,:loop_count], label = 'y error')
    plt.plot(state_list[12,:loop_count], error_list[2,:loop_count], label = 'z error')
    plt.title("positional errors")
    plt.xlabel('s')
    plt.ylabel('m')
    plt.legend()
    
    # plot yaw error
    plt.figure()
    plt.plot(state_list[12,:loop_count], error_list[3,:loop_count], label = 'yaw')
    plt.title("yaw error")
    plt.xlabel('s')
    plt.ylabel('radians')
    plt.legend()
    
    #plot motor speeds 
    plt.figure()
    plt.plot(state_list[12,:loop_count], motor_speed_list[0,:loop_count], label = 'm1')
    plt.plot(state_list[12,:loop_count], motor_speed_list[1,:loop_count], label = 'm2')
    plt.plot(state_list[12,:loop_count], motor_speed_list[2,:loop_count], label = 'm3')
    plt.plot(state_list[12,:loop_count], motor_speed_list[3,:loop_count], label = 'm4')
    plt.title("motor speeds")
    plt.xlabel('s')
    plt.ylabel('RPM')
    plt.legend()
    

if __name__ == "__main__":
    
    simulate(True)
  
    
    

