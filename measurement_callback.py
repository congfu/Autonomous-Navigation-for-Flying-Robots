import numpy as np
from plot import plot_trajectory
from math import sin, cos

class UserCode:
    def __init__(self):
        self.position = np.array([[0], [0]])
        
    def measurement_callback(self, t, dt, navdata):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param navdata: measurements of the quadrotor
        '''
        
        # TODO: update self.position by integrating measurements contained in navdata
        #mat = np.array([[cos(navdata.rotZ), -sin(navdata.rotZ), navdata.vx], [sin(navdata.rotZ), cos(navdata.rotZ), navdata.vy], [0, 0, 1]])
        #vec = mat * np.array([self.position[0,0], self.position[1, 0], 0])
        #self.position = np.array([[vec[0,0]], [vec[1, 0]]])
        pos_local=np.array([[navdata.vx*dt],[navdata.vy*dt]])
        R=np.array([[cos(navdata.rotZ),-1.0*sin(navdata.rotZ)],[sin(navdata.rotZ),cos(navdata.rotZ)]])
        pos_global=np.dot(R,pos_local)
        self.position +=pos_global
        plot_trajectory("odometry", self.position)
