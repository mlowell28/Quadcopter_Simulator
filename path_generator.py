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
    

# waypoint_1 = Waypoint(np.array([0,0,0]), 0, 0)
# waypoint_2 = Waypoint(np.array([10,10,10]), math.pi/4,10)
# waypoint_3 = Waypoint(np.array([14,2,12]), math.pi/2, 20)

# mypath = Path([waypoint_1, waypoint_2, waypoint_3])

# t = np.linspace(0,20)

# x = []
# y = []
# z = []

# for t_0 in t:
#     [x_pos, y_pos, z_pos], yaw = mypath.target_position(t_0)
#     x.append(x_pos)
#     y.append(y_pos)
#     z.append(z_pos)
    

# print(mypath.target_position(20))


# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# old_time = time.time()
# red = True

# while(1 == 1):
#     if  red == True:
#         ax.clear()
#         ax.plot(x,y,z, color='blue')
#         plt.draw()
#         plt.pause(.00000001)
#         print("blue")
#         red = False
        
#     else:
#         ax.clear()
#         ax.plot(x,y,z, color='red')
#         plt.draw()
#         plt.pause(.00000001)
#         print('red')
#         red = True
        
    
#     new_time = time.time()
    
#     if new_time - old_time > 1:
#         old_time = new_time
        
#         if red == True:
#             red = False        
#         else:
#             red = True




