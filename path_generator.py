# -*- coding: utf-8 -*-
"""
Created on Sat May 22 16:50:05 2021

@author: Michael
"""
import numpy as np
import math
from scipy import interpolate

# define waypoint class which contains position, yaw and time.

class Waypoint():
    def __init__(self, position, yaw, t):
        self.position = position
        self.yaw = yaw
        self.t = t


class Path():
    
    # take in sequential waypoints along with target time, create a path 
    # function which takes in time and gives position and yaw. 

    def __init__(self, waypoints, path_type = 'linear'):
        
        self.waypoints = waypoints
        self.path_type = path_type
        self.total_time = waypoints[-1].t
         
        x_p = []
        y_p = []
        z_p = []
        t_p = []
        
        # build list to interpolate waypoints
        
        for waypoint in waypoints:
            
            [x,y,z] = waypoint.position
            x_p.append(x)
            y_p.append(y)
            z_p.append(z)
            t_p.append(waypoint.t)
            
        self.f_x = interpolate.interp1d(t_p,x_p, path_type)
        self.f_y = interpolate.interp1d(t_p,y_p, path_type)
        self.f_z = interpolate.interp1d(t_p,z_p, path_type)

    
    def target_position(self, t):
        
        # if at the start or the end, return the position of the target waypoints       

        if 0 == t:    
            return self.waypoints[0].position, self.waypoints[0].yaw
                
        if self.total_time == t:
            return self.waypoints[-1].position, self.waypoints[-1].yaw
                
        # compute yaw to always point in direction of gradient of path projected onto x/y plan
        
        dt = .1  
        if t+dt <= self.waypoints[-1].t:
            grad = [(self.f_x(t+dt) - self.f_x(t))/dt, (self.f_y(t+dt) - self.f_y(t))/dt]
            yaw = math.atan(grad[1]/grad[0])
            
        else:
            yaw = self.waypoints[-1]
                
        return [self.f_x(t), self.f_y(t), self.f_z(t)], yaw
                       
    def get_total_time(self):
        return self.total_time
    



