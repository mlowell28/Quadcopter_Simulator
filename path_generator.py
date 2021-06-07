# -*- coding: utf-8 -*-
"""
Created on Sat May 22 16:50:05 2021

@author: Michael
"""
import numpy as np
import math
from scipy import interpolate

# define step input generator which takes in position and optinal yaw.
class StepInput():
    def __init__(self, position, yaw = 0):
        self.position = position
        self.yaw = yaw 
        
    def target_position(self, t):
        target_state = np.zeros(12)
        target_state[0:3] = self.position
        target_state[8] = self.yaw
        return target_state
    
    def get_total_time(self):
        return 1000

# define waypoint class which contains position, travel time from prior waypoint
# and an optional hover time and yaw angle to use while at the waypoint.  

class Waypoint():
    def __init__(self, position, travel_time, yaw = 0):
        self.position = position
        self.yaw = yaw
        self.travel_time = travel_time
        
        
class WaypointPath():
    
    def __init__(self, waypoints, interpolate_path = False):
        
        self.waypoints = waypoints
        self.interpolate_path = interpolate_path
        total_time = 0
        
        for waypoint in waypoints:
            total_time += waypoint.travel_time 
            waypoint.total_time = total_time
            
        self.total_time = total_time
     
    def target_position(self, t):
        
        if self.interpolate_path == False:
            next_state= np.zeros(12)
            for waypoint_index in range(len(self.waypoints)):
                if self.waypoints[waypoint_index].total_time > t:   
                    next_state[0:3] = self.waypoints[waypoint_index].position
                    next_state[8] = self.waypoints[waypoint_index].yaw
                    return next_state
                
            next_state[0:3] = self.waypoints[-1].position
            next_state[8] = self.waypoints[-1].yaw
            
            return next_state
        
        # if interpolation is true, apply linear interpolation between waypoints
        
        if self.interpolate_path == True:
            
            last_state = np.zeros(12)
            next_state = np.zeros(12)
            
            target_state = np.zeros(12)
            
            for waypoint_index in range(len(self.waypoints)):
                if self.waypoints[waypoint_index].total_time > t:  
                    last_waypoint = self.waypoints[waypoint_index-1]
                    next_waypoint = self.waypoints[waypoint_index]
                    
                    last_state[0:3] = last_waypoint.position 
                    last_state[8] = last_waypoint.yaw 
                    
                    next_state[0:3] = next_waypoint.position
                    next_state[8] = next_waypoint.yaw
                    
                    # if positions are the same but yaw is different, 
                    # return  
                    
                    if np.array_equal(last_state[0:3], next_state[0:3]):
                        target_state = next_state
                        return target_state
                    
                    # else linear interpolate positions and define 
                    # yaw so that quadcopter faces in direction of yaw 
                    
                    vec = next_state - last_state
                    target_state[8] = math.atan2(vec[1], vec[0])
                    
                    # target position 
                    
                    vec = (t-last_waypoint.total_time)/(next_waypoint.total_time - last_waypoint.total_time) *(next_state - last_state) + last_state
                    target_state[0:3] = vec[0:3]
     
                    return target_state
                
            target_state[0:3] = self.waypoints[-1].position
            target_state[8] = self.waypoints[-1].yaw
            
            return target_state
                

        
    def get_total_time(self):
        return self.total_time

class SmoothPath():
    
    # continuous functions of time starting at t = 0, optional yaw.
    def __init__(self, f_x, f_y, f_z, f_yaw, run_length, timestep = None):
        
        self.f_x = f_x
        self.f_y = f_y
        self.f_z = f_z
        self.f_yaw = f_yaw
        self.run_length = run_length
        self.timestep = timestep 
        self.discrete_time = 0
          
    def target_position(self, t):
        
        target_state = np.zeros(12)
        
        if self.timestep != None:
        
            if t > self.discrete_time:
                self.discrete_time += self.timestep
            t = self.discrete_time
            
        target_state[0:3] = [self.f_x(t), self.f_y(t), self.f_z(t)]
        target_state[8] = self.wrap_angle(self.f_yaw(t))
        
        return target_state

    def get_total_time(self):
        return self.run_length
    
    def wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )

    
# path interval takes a sequences of waypoints assuming no hovering and generates
# an interpolated path based upon path type. 
   
class SmoothInterval():
        
    def __init__(self, waypoints, starting_time, path_type = 'linear'):
        
        self.waypoints = waypoints
        self.path_type = path_type
        self.starting_time = starting_time
        
        total_time = starting_time
        
        # associate absolute travel time with waypoints 
        
        for waypoint in waypoints:
            total_time += waypoint.travel_time 
            waypoint.target_time = total_time
            
        self.ending_time = total_time
        self.path_travel_time = total_time - starting_time
        
        x_p = []
        y_p = []
        z_p = []
        t_p = []
        
        if len(waypoints) >= 2: 
            for waypoint in waypoints:
                
                [x,y,z] = waypoint.position
                x_p.append(x)
                y_p.append(y)
                z_p.append(z)
                t_p.append(waypoint.target_time)
                
            self.f_x = interpolate.interp1d(t_p,x_p, path_type)
            self.f_y = interpolate.interp1d(t_p,y_p, path_type)
            self.f_z = interpolate.interp1d(t_p,z_p, path_type)
            
        else:
            self.f_x = lambda t: waypoints[0].position[0]
            self.f_y = lambda t: waypoints[0].position[1]
            self.f_z = lambda t: waypoints[0].position[2]
        
        
    def get_time_interval(self):
        return [self.starting_time, self.ending_time]
        
    def get_interval_duration(self):
        return self.path_travel_time
    def get_starting_time(self):
        return self.starting_time
    def get_ending_time(self):
        return self.ending_time
        
    def target_position(self, t):
        target_state = np.zeros(12)
        
        # compute yaw to always point in direction of gradient of path projected onto x/y plane
        
        dt = .1  
        
        if self.ending_time <= t+dt:
            target_state[0:3] = self.waypoints[-1].position
            target_state[8] = self.waypoints[-1].yaw
            return target_state
         
        grad = [(self.f_x(t+dt) - self.f_x(t))/dt, (self.f_y(t+dt) - self.f_y(t))/dt]
        target_state[0:3] = [self.f_x(t), self.f_y(t), self.f_z(t)]
        
        # as long as quadcopter is not hovering in place find yaw value
        
        if grad[1] != 0 or grad[0] != 0:
            yaw = math.atan2(grad[1],grad[0]) 
            yaw = self.wrap_angle(yaw)
        
        # if it is hovering use target yaw value for waypoint
        else:
            yaw = self.waypoints[0].position
        
        target_state[8] = yaw
        return target_state
    
    def wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )

    

class Path():
    def __init__(self, waypoints, path_type = 'quadratic'):
        
        
        self.waypoints = waypoints
        self.waypoint_intervals = []
        self.path_intervals = []
        self.path_type = path_type
        
        interval = []
        
        waypoint_index = 0
        for waypoint_index in range(len(waypoints)):
            if waypoint_index == 0:
                interval.append(waypoints[waypoint_index])
            elif all(waypoints[waypoint_index-1].position != waypoints[waypoint_index]):
                interval.append(waypoints[waypoint_index])
            else:
                self.waypoint_intervals.append(interval)
                interval = []
                
        self.waypoint_intervals.append(interval)
        waypoint_intervals_index = 0
        
        for waypoint_intervals_index in range(len(self.waypoint_intervals)):
            if waypoint_intervals_index == 0:
                path_interval = SmoothInterval(self.waypoint_intervals[waypoint_intervals_index],0, path_type)
                self.path_intervals.append(path_interval)
            else:
                path_interval = SmoothInterval(self.waypoint_intervals[waypoint_intervals_index],self.path_intervals[-1].get_ending_time, path_type)
                self.path_intervals.append(path_interval)
                
            
    def target_position(self, t):
        for path in self.path_intervals:
            if path.get_starting_time() <= t <= path.get_ending_time():
                return path.target_position(t)
            
    def get_total_time(self):
        return self.path_intervals[-1].get_ending_time()
    
        
        
        
    


    



