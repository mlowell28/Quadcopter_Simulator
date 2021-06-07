from quadcopter_sequential import Quadcopter 
from gui_sequential import GUI
from controller_sequential import LQR_Controller
from path_generator import Waypoint, StepInput, SmoothPath, WaypointPath, Path
import numpy as np
import math
import matplotlib.pyplot as plt
import control


def simulate(use_GUI = True):
    
    print("starting simulation loop")
    
       
    # define path, second argument to waypoint is time, third is optinal yaw 
    # but is only to define ending pose 
    
    waypoint_1 = Waypoint(np.array([0,0,0]), 0)
    waypoint_2 = Waypoint(np.array([10,-10,5]), 20)
    waypoint_3 = Waypoint(np.array([10,10,5]), 20) 
    waypoint_4 = Waypoint(np.array([-10,10,5]), 20) 
    waypoint_5 = Waypoint(np.array([-10,-10,5]), 20) 
    waypoint_6 = Waypoint(np.array([10,-10,5]), 20)
    waypoint_7 = Waypoint(np.array([0,0,10]), 30)
    waypoint_8 = Waypoint(np.array([-10,10,5]), 30)
    waypoint_9 = Waypoint(np.array([0,0,0]), 30,)
    waypoint_10 = Waypoint(np.array([0,0,0]), 30, -1)
    waypoint_11 = Waypoint(np.array([0,0,10]), 40, 2.5)
    

    
    run_time = 450
    # define quadcopter parameters and starting position 
    q1_parameters = {'L':0.3,'r':0.1,'prop_parameters':[10,4.5],'weight':1.2, 'motor_limits':[1000,10000]}
    q1_starting_state = {'position':np.array([0,0,0]), 'linear_rate':np.array([0,0,0]), 'orientation':np.array([0,0,0]), 'angular_rate':np.array([0,0,0])}

    #mypath = StepInput([1,1,1],1)
    
    mypath = WaypointPath([waypoint_1,waypoint_2,waypoint_3,waypoint_4,waypoint_5, waypoint_6, waypoint_7, waypoint_8, waypoint_9, waypoint_10], interpolate_path = True)
    mypath = Path([waypoint_1,waypoint_2,waypoint_3,waypoint_4,waypoint_5, waypoint_6, waypoint_7, waypoint_8, waypoint_9, waypoint_10, waypoint_11])
    
    
    # f_x = lambda t: 10*math.sin(2*math.pi*t/50) #/100
    # f_y = lambda t: 10*math.cos(2*math.pi*t/50)
    # f_z = lambda t: 5 + t/50
    # f_yaw = lambda t: t/50 #
    
    # mypath = SmoothPath(f_x, f_y, f_z, f_yaw, run_time)
    
    
    # define LQR cost matrix
    # State space representation: [x y z, x_dot y_dot z_dot, theta phi gamma, omega_1, omega_2, omega_3]    
    
    Q = np.diag([100,100,100,10,10,10,1000,1000,100,1000,1000,100])  #np.diag([1,1,1,1,1,1,1,1,1,1,1,1])  #np.diag([1,1,1,10,10,10,1000,1000,1000,1000,1000,100]) 
    R = np.diag([.0001,.0001,.0001,.0001])
    
    # create controller and quadcopter
    
    q1_controller = LQR_Controller(mypath, Q, R)
    q1_quadcopter = Quadcopter(q1_parameters, q1_starting_state, q1_controller) #, sigma=[.01,.01,.005,.01,.01,.01,.001,.001,.001,.01,.01,.01])
    
    #GUI expects list of quadcopters as input 
    
    if use_GUI == True:
        print("starting gui")
        mygui = GUI([q1_quadcopter])
         
    # set step size in seconds
    
    dt = .01

    # compute approximate number of iterations to reach target
    
    loop_count = 0
    MAX_LOOP = int(run_time/dt)
    
    
    run = True
    print('starting loop')
    
    state_list = np.zeros((13,MAX_LOOP))
    target_position_list = np.zeros((4,MAX_LOOP))
    motor_speed_list = np.zeros((4,MAX_LOOP))
    error_list = np.zeros((4,MAX_LOOP))
    l2_error = np.zeros(MAX_LOOP)
    
    GUI_update_rate = 100
    GUI_loop_index = 0
    
    while(loop_count < MAX_LOOP and run == True):
        

        [state, t_now] = q1_quadcopter.update(dt)
        
        state_list[0:12, loop_count] = state
        state_list[12, loop_count] = t_now
        
        
        [m1, m2, m3, m4] = q1_quadcopter.get_motor_speeds()
        
        motor_speed_list[:,loop_count] = [m1, m2, m3, m4]
        
        target_position = mypath.target_position(t_now)
        x = target_position[0]
        y = target_position[1]
        z = target_position[2]
        yaw = target_position[8]
        
        target_position_list[:, loop_count] = [x, y, z, yaw]
        
        error_list[0, loop_count] = abs(x - state[0])
        error_list[1, loop_count] = abs(y - state[1])
        error_list[2, loop_count] = abs(z - state[2])
        
        l2_error[loop_count] = np.linalg.norm(error_list[0:3, loop_count])
        
        # compute error based upon what arc path has lowest radians
        yaw_error = state[8] - yaw
        
        if yaw_error > math.pi:
            yaw_error = -1*(2*math.pi - yaw_error)
        elif yaw_error < -1*math.pi:
            yaw_error = (2*math.pi + yaw_error)
        
        
        error_list[3, loop_count] = abs(yaw_error)
                
        GUI_loop_index +=1
        if GUI_loop_index == GUI_update_rate:
            print("time " + str(t_now))
            print("x error " + str(error_list[0,loop_count]))
            print("y error " + str(error_list[1,loop_count]))
            print("z error " + str(error_list[2,loop_count]))
            print("yaw error " + str(error_list[3,loop_count]))
            
            if use_GUI == True:
                run = mygui.update()
                GUI_loop_index = 0
    
        loop_count +=1
        
    
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
    plt.legend()
    plt.xlabel('s')
    plt.ylabel('radians')
    
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[7,:loop_count], label='true')
    plt.legend()
    plt.title("phi")
    plt.xlabel('s')
    plt.ylabel('radians')
    
    
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[9,:loop_count], label='true')
    plt.legend()
    plt.title("omega 1")
    plt.xlabel('s')
    plt.ylabel('radians')
    
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[10,:loop_count], label='true')
    plt.legend()
    plt.title("omega 2")
    plt.xlabel('s')
    plt.ylabel('radians')
    
    plt.figure()
    plt.plot(state_list[12,:loop_count], state_list[11,:loop_count], label='true')
    plt.legend()
    plt.title("omega 3")
    plt.xlabel('s')
    plt.ylabel('radians')
    
    #plot positional errors
    plt.figure()
    plt.plot(state_list[12,:loop_count], error_list[0,:loop_count], label = 'x error')
    plt.plot(state_list[12,:loop_count], error_list[1,:loop_count], label = 'y error')
    plt.plot(state_list[12,:loop_count], error_list[2,:loop_count], label = 'z error')
    plt.title("positional errors")
    plt.xlabel('s')
    plt.ylabel('m')
    plt.legend()
    
    # plot l2 position errors:
    plt.figure()
    plt.plot(state_list[12,:loop_count], l2_error[:loop_count], label = 'l2 error')
    plt.title("l2 position error")
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
  
    
    

