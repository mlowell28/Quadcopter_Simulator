from quadcopter_sequential import Quadcopter 
from gui_sequential import GUI
from controller_sequential import LQR_Controller
from path_generator import Path, Waypoint
import signal
import time
import numpy as np
import math


def simulate():
    
    print("starting simulation loop")
    
       
    # define path
    
    waypoint_1 = Waypoint(np.array([0,0,0]), 0, 0)
    waypoint_2 = Waypoint(np.array([10,10,10]), math.pi/4,600)
    waypoint_3 = Waypoint(np.array([14,2,12]), math.pi/2, 600)
    
    # define quadcopter parameters and starting position 
    q1_parameters = {'L':0.3,'r':0.1,'prop_parameters':[10,4.5],'weight':1.2, 'motor_limits':[4000,9000]}
    q1_starting_state = {'position':np.array([0,0,0]), 'linear_rate':np.array([0,0,0]), 'orientation':np.array([0,0,0]), 'angular_rate':np.array([0,0,0])}

    
    # generate interpolated path 
    
    mypath = Path([waypoint_1, waypoint_2, waypoint_3])
    
    # define LQR cost matrix
    
    Q = np.diag([100,100,100,100,100,100,100,100,100,100,100,100])
    R = np.diag([1,1,1,1])
    
    # create controller and quadcopter
    
    q1_controller = LQR_Controller(mypath, Q, R)
    q1_quadcopter = Quadcopter(q1_parameters, q1_starting_state, q1_controller)
    
    #GUI expects list of quadcopters as input 
    
    print("starting gui")
    mygui = GUI([q1_quadcopter])
    
    
    dt = 1/600
    t = 0
    iteration_count = 0
    last_time = time.time()
    
    run = True
    print('starting loop')
    while(run == True):
        q1_quadcopter.update(dt)
        t += dt
        iteration_count += 1
        
        if iteration_count == 10:
            run = mygui.update()
            iteration_count = 0       


if __name__ == "__main__":
    
    simulate()
  
    
    

