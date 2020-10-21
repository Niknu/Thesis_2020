#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import rospy

from std_msgs.msg import Float64MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


from sensor_msgs.msg import NavSatFix, Imu     #From the /mavros/global_position/global or mavros/global_position/raw/fix
from math import *

from ecef2geodtic_def import * # This is for running the function GEOToNED


static_Q = True


class Pos_estimation():
    def __init__(self):

        self.scale_QVar = 0.1

        self.dataGPS = None
        self.dataIMU = None
        self.t_current = None
        self.t_prev = None
        self.FirstRun_dt = True
        self.FirstRun_KF = True

        self.GPS_on = False
        self.IMU_on = False

        self.GPS_start = True
        self.GPS_0 = np.zeros([3,1])#,dtype="float32")
        self.NED = np.zeros([3,1])#,dtype="float32")
        self.R = np.zeros([3,3])#,dtype="float32")

        self.IMU_data = np.zeros([3,1])#,dtype="float32")

        ## ---- The Offset is calculated from previsly measurments --- #
        self.imu_x_offset = -0.1560196823083865
        self.imu_y_offset = -0.12372256601510975
        self.imu_z_offset = 9.8038682107820652
        self.dt=0.1
        self.sensorData = np.zeros([9,1])#,dtype="float32")

        # the publish data-type 
        #self.estPos_data = Float64MultiArray()
        #self.estPos_data = numpy_msg()

        self.I = np.eye(3)#,dtype="float32")
        self.ZERO = np.zeros((3,3))#,dtype="float32")
        
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


        if static_Q is True:
            self.klmFilt.Q = np.eye(9)*self.scale_QVar
        else: 
            self.klmFilt.Q = Q_discrete_white_noise(dim=3, dt=self.dt ,block_size=3,order_by_dim=False)

        

        #self.klmFilt.P *= 1000.0
        self.klmFilt.R = self.klmFilt.H

        self.klmFilt.inv =  np.linalg.pinv   #The inv method is changed # This is done because it will gives a "linalgError: singular matrix"

        #init the node and subscribe to the GPS adn IMU
        rospy.init_node('KF_pos_estimation',anonymous=True)
        rospy.Subscriber('/mavros/global_position/raw/fix',NavSatFix,self.gps_data_load)
        rospy.Subscriber('/mavros/imu/data',Imu,self.imu_data_load)

        # creating the publisher
        self.estPos_pub = rospy.Publisher('/KF_pos_est',numpy_msg(Floats),queue_size=1)
        #rospy.Rate(10) # 10Hz 
        rospy.spin()



    def gps_data_load(self,data):
        self.dataGPS = data
        self.GPS_on = True
        self.update_dt_A_Q_R(self.dataGPS)
        self.GPS_on = False

        #Transform to local coordinates, where z is possitive downwards
        self.GeoToNED_old(self.dataGPS.latitude,self.dataGPS.longitude,self.dataGPS.altitude)

        self.sensorData[0] = self.NED[0]
        self.sensorData[1] = self.NED[1]
        self.sensorData[2] = self.NED[2]

        if self.FirstRun_KF is True:
            self.klmFilt.x = self.sensorData
            self.klmFilt.P *= 1000.0
            self.FirstRun_KF = False

        #Correct KF
        self.klmFilt.update(self.sensorData)
        x_hat = self.klmFilt.x
        
        # send data
        self.pub_xhat_data(x_hat)

    def imu_data_load(self,data):
        self.dataIMU = data
        self.IMU_on = True
        self.update_dt_A_Q_R(self.dataIMU)
        self.IMU_on = False

        self.sensorData[6] = self.dataIMU.linear_acceleration.x - self.imu_x_offset
        self.sensorData[7] = self.dataIMU.linear_acceleration.y - self.imu_y_offset
        self.sensorData[8] = self.dataIMU.linear_acceleration.z - self.imu_z_offset

        if self.FirstRun_KF is True:
            self.klmFilt.x = self.sensorData
            self.klmFilt.P *= 1000.0
            self.FirstRun_KF = False

        #Predict KF
        self.klmFilt.predict()
        
        #Update KF
        self.klmFilt.update(self.sensorData)


        #Correct KF
        self.klmFilt.update(self.sensorData)
        x_hat = self.klmFilt.x

        # send data
        self.pub_xhat_data(x_hat)

    def pub_xhat_data(self,data):
        #print(' ')
        Acc_cali_raw = np.array([self.sensorData[6],self.sensorData[7],self.sensorData[8]])
        data = np.append(data,[self.NED,Acc_cali_raw]) # Add the input data to the KF into the published data (for debugging)
        data = np.float32(data)
        #print(data)
        self.estPos_pub.publish(data)


    def update_dt_A_Q_R(self,data):
        self.update_dt(data)
        self.update_klm_A()
        if static_Q is not True:
            self.update_klm_Q()

        if self.GPS_on is True:
            R_gps = np.copy(self.dataGPS.position_covariance)
            R_gps = np.reshape(R_gps,(3,3))
            #print('R_gps shape= ',R_gps)
            #print(' ')
            self.update_Rgps(R_gps)
        elif self.IMU_on is True:
            R_acc = np.copy(self.dataIMU.linear_acceleration_covariance)
            R_acc = np.reshape(R_acc,(3,3))
            #print('R_acc shape= ',R_acc)
            #print(' ')
            self.update_Racc(R_acc)

    def update_dt(self,data):
        t_secs_new = data.header.stamp.secs
        t_nsecs_new = data.header.stamp.nsecs*1.0e-9            # Convert from nsecs to secs

        t_current = t_secs_new + t_nsecs_new
        
        if self.FirstRun_dt is not True:
            self.dt = t_current - self.t_prev                   #Find the time difference (delta t)

        self.t_prev = t_current

        self.FirstRun_dt = False

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

    def GeoToNED_new(self,lat_in,lon_in,alt_in):
        
        lat = d2r * lat_in
        lon = d2r * lon_in
        h = d2r * alt_in


        a = 6378137.0 #[m]
        b = 6356752.3 #[m]
        f = 0.00335281
        E = 0.0818

        N = a/(sqrt(1.0-E**2.0*sin(lat)**2.0))
        
        XYZ = np.zeros([3,1])#,dtype="float32")

        XYZ[0] = (N+h)*cos(lat)*cos(lon) #x
        XYZ[1] = (N+h)*cos(lat)*sin(lon) #y
        XYZ[2] = (N*(1-E**2)+h)*sin(lat) #z

        if self.GPS_start is True:
            self.GPS_0[0] = XYZ[0]
            self.GPS_0[1] = XYZ[1]
            self.GPS_0[2] = XYZ[2]
            print('------Resets the Zero pos --- GPS')
            self.GPS_start =  False

        self.rotMat_NED(lat,lon)

        self.NED = np.dot(self.R,(XYZ-self.GPS_0))

    def GeoToNED_old(self,lat_in,lon_in,alt_in):

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

        XYZ = np.zeros([3,1])#,dtype="float32")
        XYZ[0] = d * coslon                 #x
        XYZ[1] = d * sinlon                 #y
        XYZ[2] = (p1mee * N + alt) * sinlat #z


        if self.GPS_start is True:
            self.GPS_0[0] = XYZ[0]
            self.GPS_0[1] = XYZ[1]
            self.GPS_0[2] = XYZ[2]
            #print('-----RESETS GPS_0')
            self.GPS_start =  False

        self.rotMat_NED(lat,lon)

        #print('GPS_0= ',self.GPS_0)
        #print('XYZ= ',XYZ)
        #print('xyz-gps_0=',XYZ-self.GPS_0)
        #print('Rot_mat= ',self.R)
        self.NED = np.dot(self.R,(XYZ-self.GPS_0))    # Have changed the sign in for the z part in R!!!

        #print('NED =',self.NED)
        #print(' ')

    def rotMat_NED(self,lat,lon):
        self.R = np.array([
            [-sin(lat)*cos(lon),-sin(lat)*sin(lon),cos(lat)],
            [-sin(lon),cos(lon),0],
            [cos(lat)*cos(lon),cos(lat)*sin(lon), sin(lat)]   ### Change have change compared to the original formula
        ])#,dtype="float32")



if __name__ == '__main__':
    try:
        Pos_estimation()
    except rospy.ROSInterruptException as e:
        print (e)


