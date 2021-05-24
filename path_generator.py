# -*- coding: utf-8 -*-
"""
Created on Sat May 22 16:50:05 2021

@author: Michael
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
from scipy import interpolate
import signal
import sys
import time

class Waypoint():
    def __init__(self, position, yaw, t):
        self.position = position
        self.yaw = yaw
        self.t = t


class Path():
    
    # take in waypoints along with target time, create a path function which 
    # takes in time and gives position, yaw and approximate orientation
    
    def __init__(self, waypoints, path_type = 'linear'):
        
        self.waypoints = waypoints
        self.path_type = path_type
        self.total_time = waypoints[-1].t
    
    def target_position(self, t):
        
        if t == 0:
            return [self.waypoints[0].position, self.waypoints[0].yaw]  
        
        if t >= self.waypoints[len(self.waypoints)-1].t:
            return [self.waypoints[-1].position, self.waypoints[-1].yaw]  
        
        if self.path_type == 'linear': 
        
            for i in range(len(self.waypoints)-1):
                if self.waypoints[i].t <= t and self.waypoints[i+1].t >= t:
                    
                    f_x = interpolate.interp1d([self.waypoints[i].t, self.waypoints[i+1].t], [self.waypoints[i].position[0], self.waypoints[i+1].position[0]])
                    f_y = interpolate.interp1d([self.waypoints[i].t, self.waypoints[i+1].t], [self.waypoints[i].position[1], self.waypoints[i+1].position[1]])
                    f_z = interpolate.interp1d([self.waypoints[i].t, self.waypoints[i+1].t], [self.waypoints[i].position[2], self.waypoints[i+1].position[2]])    
                    
                    
                    # if within .1 of next waypoint
                    
                    if np.linalg.norm((np.array([f_x(t), f_y(t), f_z(t)]) - self.waypoints[i+1].position)) < .1:
                        return np.array([f_x(t), f_y(t), f_z(t)]), self.waypoints[i+1].yaw                    
                    
                    # else set yaw towards next waypoint 
                    
                    [x,y,z] = self.waypoints[i+1].position - self.waypoints[i].position
                    yaw = math.atan(y/x)
                    
                    return [f_x(t), f_y(t), f_z(t)], yaw
                       
    def get_total_time(self):
        return self.total_time
    



