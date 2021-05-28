import numpy as np
import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import sys

class GUI():

    def __init__(self, quads, real_time = True):
        self.quads = quads
        self.fig = plt.figure()
        self.ax = Axes3D.Axes3D(self.fig)
        self.ax.set_xlim3d([-20.0, 20.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-20.0, 20.0])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, 20.0])
        self.ax.set_zlabel('Z')
        self.ax.set_title('Quadcopter Simulation')
        self.init_plot()
        
        if real_time == True:     
            self.fig.canvas.mpl_connect('key_press_event', self.keypress_routine)
        self.run = True
        

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

    def init_plot(self):
        for quad in self.quads:
            quad.graphics = {}
            quad.graphics['l1'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
            quad.graphics['l2'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
            quad.graphics['hub'], = self.ax.plot([],[],[],marker='o',color='green', markersize=6,antialiased=False)
            quad.graphics['trajectory'], = self.ax.plot([],[],[], color='purple', linewidth=1, antialiased=False)

    def update(self):
        for quad in self.quads:
            R = self.rotation_matrix(quad.get_orientation())
            L = quad.parameters['L']
            points = np.array([ [-2*L,0,0], [2*L,0,0], [0,-2*L,0], [0,2*L,0], [0,0,0], [0,0,0] ]).T
            points = np.dot(R,points)
            
            position = quad.get_position()
            
            points[0,:] += position[0]
            points[1,:] += position[1]
            points[2,:] += position[2]
            
            quad.graphics['l1'].set_data(points[0,0:2],points[1,0:2])
            quad.graphics['l1'].set_3d_properties(points[2,0:2])
            quad.graphics['l2'].set_data(points[0,2:4],points[1,2:4])
            quad.graphics['l2'].set_3d_properties(points[2,2:4])
            quad.graphics['hub'].set_data(points[0,5],points[1,5])
            quad.graphics['hub'].set_3d_properties(points[2,5])
            
            # plot path 
            
            mypath = quad.get_target_path()
            if mypath != None:
                total_time = mypath.get_total_time()
                
                t_span = np.linspace(0, total_time, 1000)
                
                x_p = np.zeros(1000)
                y_p = np.zeros(1000)
                z_p = np.zeros(1000)
                
                index = 0
                for t in t_span:
    
                    [x, y, z], yaw = mypath.target_position(t)       
                    
                    x_p[index] = x
                    y_p[index] = y
                    z_p[index] = z
                    
                    index +=1
    
                quad.graphics['trajectory'].set_data(x_p, y_p)
                quad.graphics['trajectory'].set_3d_properties(z_p) 
            plt.pause(.00000000001)    
            return self.run
        
    def generate_image(self):
        for quad in self.quads:
            R = self.rotation_matrix(quad.get_orientation())
            L = quad.parameters['L']
            points = np.array([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
            points = np.dot(R,points)
            
            position = quad.get_position()
            
            points[0,:] += position[0]
            points[1,:] += position[1]
            points[2,:] += position[2]
            
            quad.graphics['l1'].set_data(points[0,0:2],points[1,0:2])
            quad.graphics['l1'].set_3d_properties(points[2,0:2])
            quad.graphics['l2'].set_data(points[0,2:4],points[1,2:4])
            quad.graphics['l2'].set_3d_properties(points[2,2:4])
            quad.graphics['hub'].set_data(points[0,5],points[1,5])
            quad.graphics['hub'].set_3d_properties(points[2,5])
            
            # plot path 
            
            mypath = quad.get_target_path()
            if mypath != None:
                total_time = mypath.get_total_time()
                
                t_span = np.linspace(0, total_time, 100)
                
                x_p = np.zeros(100)
                y_p = np.zeros(100)
                z_p = np.zeros(100)
                
                index = 0
                for t in t_span:
    
                    [x, y, z], yaw = mypath.target_position(t)       
                    
                    x_p[index] = x
                    y_p[index] = y
                    z_p[index] = z
                    
                    index +=1
    
                quad.graphics['trajectory'].set_data(x_p, y_p)
                quad.graphics['trajectory'].set_3d_properties(z_p) 
                
        return [quad.graphics['l1'], quad.graphics['l2'], quad.graphics['hub'], quad.graphics['trajectory']]
 
    def keypress_routine(self,event):
        
        sys.stdout.flush()
        if event.key == 'q':
            self.run = False
        elif event.key == 'x':
            y = list(self.ax.get_ylim3d())
            y[0] += 0.2
            y[1] += 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'w':
            y = list(self.ax.get_ylim3d())
            y[0] -= 0.2
            y[1] -= 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'd':
            x = list(self.ax.get_xlim3d())
            x[0] += 0.2
            x[1] += 0.2
            self.ax.set_xlim3d(x)
        elif event.key == 'a':
            x = list(self.ax.get_xlim3d())
            x[0] -= 0.2
            x[1] -= 0.2
            self.ax.set_xlim3d(x)
            

