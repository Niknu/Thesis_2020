#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix, Imu     #From the /mavros/global_position/global or mavros/global_position/raw/fix
from math import *




class Pos_estimation():
    def __init__(self):

        self.dataGPS = None
        self.dataIMU = None
        self.t_current = None
        self.t_prev = None
        self.FirstRun_dt = True

        self.GPS_start = True
        self.GPS_0 = np.zeros([3,1])
        self.NED = np.zeros([3,1])

        self.dt=0.1

        self.estPos_data = Float32MultiArray()

        self.I = np.eye(3,dtype=float)
        self.ZERO = np.zeros((3,3))
        
        pv = self.I*self.dt
        pa = self.I*0.5*self.dt**2

        P = np.hstack((self.I,pv,pa))
        V = np.hstack((self.ZERO,self.I,pv))
        a = np.hstack((self.ZERO,self.ZERO,self.I))

        self.klmFilt = KalmanFilter(dim_x=9,dim_z=9) 
        self.klmFilt.F = np.vstack((P,V,a))

        C_gps = np.hstack((self.I,self.ZERO,self.ZERO))
        C_vel = np.hstack((self.ZERO,self.ZERO,self.ZERO))
        C_acc = np.hstack((self.ZERO,self.ZERO,self.I))
        self.klmFilt.H = np.vstack((C_gps,C_vel,C_acc))
        #klmFilt.Q = np.eye(klmFilt.F.shape[0],dtype=float)
        self.klmFilt.Q = Q_discrete_white_noise(dim=3, dt=self.dt ,block_size=3,order_by_dim=False)

        self.klmFilt.P *= 1000.0
        self.klmFilt.R = self.klmFilt.H

        print(self.klmFilt.x)

        #init the node and subscribe to the GPS adn IMU
        rospy.init_node('KF_pos_estimation',anonymous=True)
        rospy.Subscriber('/mavros/global_position/raw/fix',NavSatFix,self.gps_data_load)
        rospy.Subscriber('/mavros/imu/data',Imu,self.imu_data_load)

        # creating the publisher
        self.estPos_pub = rospy.Publisher('/KF_pos_est',Float32MultiArray,queue_size=1)
        rospy.Rate(10) # 10Hz 

    def gps_data_load(self,data):
        self.dataGPS = data
        self.update_dt(data)

    def imu_data_load(self,data):
        self.dataIMU = data
        self.update_dt(data)




    def update_dt(self,data):
        t_secs_new = data.header.stamp.secs
        t_nsecs_new = data.header.stamp.nsecs*1.0e-9            # Convert from nsecs to secs

        t_current = t_secs_new + t_nsecs_new
        
        if self.FirstRun_dt is not True:
            self.dt = t_current - self.t_prev                       #Find the time difference (delta t)

        self.t_prev = t_current

        self.FirstRun_dt = False

    def update_dt_A_R(self,dtNew):
        self.update_dt(dtNew)
        self.update_klm_A()
        self.update_klm_Q()




    def update_klm_A(self):
        pv = self.I*self.dt
        pa = self.I*0.5*self.dt**2
        P = np.hstack((self.I,pv,pa))
        V = np.hstack((self.ZERO,self.I,pv))
        a = np.hstack((self.ZERO,self.ZERO,self.I))
        self.klmFilt.F = np.vstack((P,V,a))

    def update_klm_Q(self):
        self.klmFilt.Q = Q_discrete_white_noise(dim=3, dt=self.dt ,block_size=3,order_by_dim=False)

    def update_Rgps(self, Rgps):
        self.klmFilt.R[0:3,0:3] = Rgps

    def update_Racc(self, Racc):
        self.klmFilt.R[6:9,6:9] = Racc


    def GeoToNED(self,lat_in,lon_in,alt_in):

        lat = lat_in
        lon = lon_in
        alt = alt_in

        lat = d2r * lat
        lon = d2r * lon
        alt = d2r * alt

        coslat = cos(lat)
        sinlat = sin(lat)
        coslon = cos(lon)
        sinlon = sin(lon)

        N = aadc / sqrt(coslat * coslat + bbdcc)
        d = (N + alt) * coslat

        XYZ = np.zeros([3,1])

        XYZ[0] = d * coslon                 #x
        XYZ[1] = d * sinlon                 #y
        XYZ[2] = (p1mee * N + alt) * sinlat #z


        if self.GPS_start is True:
            self.GPS_0[0] = x
            self.GPS_0[1] = y
            self.GPS_0[2] = z
            self.GPS_start =  False

        R = self.rotMat_NED(lat,lon)

        self.NED = R @ (XYZ-self.GPS_0)

            

    def rotMat_NED(self,lat,lon):
        R = np.([
            [-sin(lat)*cos(lon),-sin(lat)*sin(lon),cos(lat)],
            [-sin(lon),cos(lon),0],
            [-cos(lat)*cos(lon),-cos(lat)*sin(lon), -sin(lat)]
        ])

        return R

        


if __name__ == '__main__':
    try:
        Pos_estimation()
    except rospy.ROSInterruptException as e:
        print (e)


