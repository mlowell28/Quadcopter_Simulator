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
        return self.position, self.yaw
    
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
    
    def __init__(self, waypoints):
        
        self.waypoints = waypoints
        total_time = 0
        
        for waypoint in waypoints:
            total_time += waypoint.travel_time 
            waypoint.total_time = total_time
            
        self.total_time = total_time
     
    def target_position(self, t):
        
        for waypoint in self.waypoints:
            if waypoint.total_time > t:
                return waypoint.position, waypoint.yaw
        
        return self.waypoints[-1].position, self.waypoints[-1].yaw
        
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
        
        if self.timestep != None:
        
            if t > self.discrete_time:
                self.discrete_time += self.timestep
            t = self.discrete_time
            
            
        return [self.f_x(t), self.f_y(t), self.f_z(t)], self.f_yaw(t)

    def get_total_time(self):
        return self.run_length
    
    def wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )

    
# path interval takes a sequences of waypoints assuming no hovering and generates
# an interpolated path based upon path type. 
   
# class PathInterval():
    
#     def __init__(self, waypoints, starting_time = 0, path_type = 'linear', yaw_offset = math.pi/4):
        
#         self.yaw_offset = yaw_offset
        
#         self.waypoints = waypoints
#         self.starting_time = starting_time
           
#         x_p = []
#         y_p = []
#         z_p = []
#         t_p = []
        
#         self.ending_time = self.starting_time
        
#         for waypoint in waypoints:
            
#             [x,y,z] = waypoint.position
#             x_p.append(x)
#             y_p.append(y)
#             z_p.append(z)
#             self.ending_time = waypoint.t_trv - waypoints[0].t_trv + self.ending_time
#             t_p.append(self.ending_time)
            
#         self.f_x = interpolate.interp1d(t_p,x_p, path_type)
#         self.f_y = interpolate.interp1d(t_p,y_p, path_type)
#         self.f_z = interpolate.interp1d(t_p,z_p, path_type)
        
        
#     def get_time_interval(self):
#         return [self.starting_time, self.ending_time]
        
#     def get_interval_duration(self):
#         return self.ending_time - self.starting_time
    
#     def get_total_time(self):
#         return self.ending_time
        
#     def target_position(self, t):
        
#         # compute yaw to always point in direction of gradient of path projected onto x/y plane
        
#         dt = .1  
        
#         if self.ending_time <= t+dt:
#             return self.waypoints[-1].position, self.waypoints[-1].yaw
         
#         grad = [(self.f_x(t+dt) - self.f_x(t))/dt, (self.f_y(t+dt) - self.f_y(t))/dt]
#         yaw = math.atan2(grad[1],grad[0]) 
#         yaw = self.wrap_angle(yaw + self.yaw_offset)

#         return [self.f_x(t), self.f_y(t), self.f_z(t)], yaw
    
#     def wrap_angle(self,val):
#         return( ( val + np.pi) % (2 * np.pi ) - np.pi )

    

# class Path():
#     def __init__(self, waypoints):
        
        
#         self.waypoints = waypoints
#         self.Path_Intervals = []
#         interval = []
        
#         for waypoint in waypoints:
            
        
#             interval.append(waypoint)
#             if waypoint.t_hvr != 0:
#                 self.Path_Intervals.append(path_Interva(interval))
#                 self.Path_intervals.append()
#                 se
         
                

        
#     def target_position(self, t):
        
#         for smooth_path in self.smooth_paths:
#             t_l, t_h = smooth_path.get_time_interval()
            
#             if t_l <= t < t_h:
#                 return smooth_path.target_position(self, t)
            
        
    


    



