#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import rospy
from message_filters import *  # For subscribe to multiple topics and do one thing


from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
#from rospy.numpy_msg import numpy_msg
#from rospy_tutorials.msg import Floats


from sensor_msgs.msg import NavSatFix, Imu, MagneticField    #From the /mavros/global_position/global or mavros/global_position/raw/fix
from math import *

from ecef2geodtic_def import * # This is for running the function GEOToNED
from madgwickahrs import MadgwickAHRS
from quaternion import Quaternion 


static_Q = True

class ES_EKF():
    def __init__(self):

        ###################
        # For time update #
        ###################
        self.t_current = None
        self.t_prev = None
        self.FirstRun_dt = True
        self.dt = 0.0


        #############################
        # Kalman Filter init params #
        #############################
        #self.P_num = np.zeros((3,1))                            # position numinal state
        #self.V_num = np.zeros((3,1))                            # speed numinal state
        #self.quat_num = Quaternion(1.0,0.0,0.0,0.0)             # quaterion numinal state
        #self.OMEGA = np.zeros((4,4))                            # skrew matrix for the angular velocity, used in the norminal state for quaternions
        self.sensorData = np.zeros([15,1])
        self.acc_bias = np.zeros([3,1]) # Maybe the *_bias isn't used check it.
        self.gyro_bias = np.zeros([3,1])
        self.g = np.array([[0,0,-9.803868]]).reshape((3,-1))
        self.acc_data = np.zeros([3,1])
        self.gyro_data = np.zeros([3,1])
        self.P = np.eye(15)*1000.0                              # Covariance Matrix for KF
        self.Q = np.ones((15,1))                                # Unsernety for the prediction - Indipendent! therefore dim=1
        #self.R = np.zeros(15) # moved below H        # <.-- The GPS only or the quat also(madwick filter) ?   # Unsernety for the sensors
        self.u_delta_h = np.zeros((6,1))                        # The bias update for the sensors
        self.u = np.zeros((6,1))                                # The vector with the sensor input + bias
        self.x_h = np.zeros((10,1))                             # State vector
        self.x_h[6:10] = Quaternion(1.0,0.0,0.0,0.0)._q
        self.z = np.zeros((15,1))                               # Vector holing the error of the state and the bias from the sensors
        self.H = np.zeros((15,15))                              # Only used for the GPS signal
        self.H[0,0] = 1
        self.H[1,1] = 1
        self.H[2,2] = 1
        self.R = self.H      # <--- Remember that R cov matrix can also come from GPS
        self.F = None # More details, look 'def Fmatrix_update() ' 


        #######################
        # GPS transform setup #
        #######################
        self.GPS_start = True
        self.GPS_0 = np.zeros([3,1])
        self.NED = np.zeros([3,1])
        self.rot_GPS = np.zeros([3,3]) # The output from the function in rotMat_NED or ENU


        # --- Madgwick filter --- #
        #self.beta = 0.334 # This is from his own paper where he point out where it's optimal
        #self.madFilt=MadgwickAHRS(beta=self.beta,sampleperiod=1/self.dt)


        #############
        # ROS setup #
        #############

        #init the node and subscribe to the GPS adn IMU
        rospy.init_node('ES_EKF',anonymous=True)
        rospy.Subscriber('/mavros/global_position/raw/fix',NavSatFix,self.correct_KF_GPS)
        rospy.Subscriber('/mavros/imu/data_raw',Imu,self.prediction_KF_IMU)
        #self.mag_data = Subscriber('/mavros/imu/mag',MagneticField)
        #self.imu_data = Subscriber('/mavros/imu/data',Imu)
        #self.madgwick_sub = ApproximateTimeSynchronizer([self.mag_data,self.imu_data],1,1)
        #self.madgwick_sub.registerCallback(self.madgwickFilter_callback)
        

        # creating the publisher
        #self.estPos_pub = rospy.Publisher('/KF_pos_est',numpy_msg(Floats),queue_size=1)
        self.estPos_pub_multarry = rospy.Publisher('/ES_EKF_output',Float64MultiArray,queue_size=1)
        #rospy.Rate(10) # 10Hz 
        rospy.spin()

    '''
    def madgwickFilter_callback(self,magData,imuData):
    
        acc = np.empty(3)
        gyro = np.empty(3)
        mag = np.empty(3)

        acc[0] =imuData.linear_acceleration.x
        acc[1] =imuData.linear_acceleration.y
        acc[2] =imuData.linear_acceleration.z

        gyro[0] =imuData.angular_velocity.x
        gyro[1] =imuData.angular_velocity.y
        gyro[2] =imuData.angular_velocity.z

        mag[0] = magData.magnetic_field.x
        mag[0] = magData.magnetic_field.y
        mag[0] = magData.magnetic_field.z

        __ , self.qDot = self.madFilt.update(gyroscope=gyro,accelerometer=acc,magnetometer=mag) # the output is the quat(not used) and qdot

        print(self.qDot[0],self.qDot[1],self.qDot[2],self.qDot[3])
    '''

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

        #self.klmFilt.x = self.sensorData

        #Predict KF
        self.klmFilt.predict()

        #Correct KF
        self.klmFilt.update(self.sensorData,H=self.H_GPS)
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

        #Predict KF
        self.klmFilt.predict()
        
        #self.klmFilt.x = self.sensorData
        #Correct/update KF
        self.klmFilt.update(self.sensorData,H=self.H_acc)
        x_hat = self.klmFilt.x

        # send data
        self.pub_xhat_data(x_hat)

    def pub_xhat_data(self,data):

        # Hint: To see the output on rqt_plot (https://answers.ros.org/question/226584/how-do-i-use-rqt_plot-to-subscribe-to-std_msgs/)


        Acc_cali_raw = np.array([self.sensorData[6],self.sensorData[7],self.sensorData[8]])
        data = np.append(data,[self.NED,Acc_cali_raw]) # Add the input data to the KF into the published data (for debugging)

        layout = self.init_multdata()
        data_1 = Float64MultiArray(layout=layout,data=data)

        self.estPos_pub_multarry.publish(data_1)

    def init_multdata(self):
        msg = MultiArrayLayout()

        msg.data_offset = 0

        msg.dim = [MultiArrayDimension()]

        msg.dim[0].label= "state_estimation"
        msg.dim[0].size = 15
        msg.dim[0].stride = 15

        return msg

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
        R_blank = np.zeros((9,9)) # this is done to be sure that the other part isn't taking into account
        self.klmFilt.R = R_blank
        self.klmFilt.R[0:3,0:3] = Rgps

    def update_Racc(self, Racc):
        R_blank = np.zeros((9,9)) # This is done to be sure that the other part isn't taking into account
        self.klmFilt.R = R_blank
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
        
        XYZ = np.zeros([3,1])

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

        self.NED = np.dot(self.rot_GPS,(XYZ-self.GPS_0))

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

        XYZ = np.zeros([3,1])
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
        #self.rotMat_ENU(lat,lon)
        #print('GPS_0= ',self.GPS_0)
        #print('XYZ= ',XYZ)
        #print('xyz-gps_0=',XYZ-self.GPS_0)
        #print('Rot_mat= ',self.R)

        # NED = rot_GPS*(XYZ-GPS_0)
        self.NED = np.dot(self.rot_GPS,(XYZ-self.GPS_0))    # Have changed the sign in for the z part in R!!!

        #print('NED =',self.NED)
        #print(' ')

    def rotMat_NED(self,lat,lon):
        self.rot_GPS = np.array([
            [-sin(lat)*cos(lon),-sin(lat)*sin(lon),cos(lat)],
            [-sin(lon),cos(lon),0],
            [-cos(lat)*cos(lon),-cos(lat)*sin(lon), -sin(lat)]   ### Change have change compared to the original formula
        ])

    def rotMat_ENU(self,lat,lon):
        self.rot_GPS = np.array([
            [-sin(lon),cos(lon),0],
            [-sin(lat)*cos(lon),-sin(lat)*sin(lat),cos(lat)],
            [cos(lat)*cos(lon),cos(lat)*sin(lon),sin(lat)],
        ])

    ############
    # NEW part #
    ############

    def pub_xhat_data_NEW(self,data):

        # Hint: To see the output on rqt_plot (https://answers.ros.org/question/226584/how-do-i-use-rqt_plot-to-subscribe-to-std_msgs/)

        #Acc_cali_raw = np.array([self.sensorData[6],self.sensorData[7],self.sensorData[8]])
        #data = np.append(data,[self.NED,Acc_cali_raw]) # Add the input data to the KF into the published data (for debugging)

        layout = self.init_multdata_NEW(data)
        data_1 = Float64MultiArray(layout=layout,data=data)

        self.estPos_pub_multarry.publish(data_1)

        print self.P
        print '----'

    def init_multdata_NEW(self,data):
        msg = MultiArrayLayout()

        msg.data_offset = 0

        msg.dim = [MultiArrayDimension()]

        msg.dim[0].label= "state_estimation"
        msg.dim[0].size = data.shape[0]
        msg.dim[0].stride = data.shape[0]

        return msg

    def prediction_KF_IMU(self,data_IMU):
        self.update_dt(data_IMU)

        # Load data and correct data with the bias
        self.acc_data[0] = data_IMU.linear_acceleration.x + self.u_delta_h[0]
        self.acc_data[1] = data_IMU.linear_acceleration.y + self.u_delta_h[1]
        self.acc_data[2] = data_IMU.linear_acceleration.z + self.u_delta_h[2]
        self.gyro_data[0] = data_IMU.angular_velocity.x + self.u_delta_h[3]
        self.gyro_data[1] = data_IMU.angular_velocity.y + self.u_delta_h[4]
        self.gyro_data[2] = data_IMU.angular_velocity.z + self.u_delta_h[5]

        quat_temp = Quaternion(w=self.x_h[6],x=self.x_h[7],y=self.x_h[8],z=self.x_h[9])

        self.cal_posVel_predict(self.acc_data,quat_temp)
        self.cal_quat_predict(self.gyro_data)

        F = self.Fmatrix_update(self.acc_data,quat_temp)

        self.cal_P_predict(F)

    def correct_KF_GPS(self,data_GPS):
        
        self.update_dt(data_GPS)
        #Correct GPS data to be NED or ENU (Decide z should point up or down) <- inside the function below
        self.GeoToNED_old(data_GPS.latitude,data_GPS.longitude,data_GPS.altitude)

        self.sensorData[0] = self.NED[0]
        self.sensorData[1] = self.NED[1]
        self.sensorData[2] = self.NED[2]

        self.cal_KF_gain()
        self.x_h_correct(self.sensorData)
        self.cal_P_correct()
        # Is it need with P reset as mention in the literature?

        self.pub_xhat_data_NEW(self.x_h)

    def cal_KF_gain(self):

        #ERROR after some time:

        # LinAlgError: SVD did not converge

        # K = P*H.T*inv(H*P*H.T+R)
        s = np.dot(np.dot(self.H,self.P),self.H.T)+self.R
        S = np.linalg.pinv(s)
        self.K = np.dot(np.dot(self.P,self.H.T),S)

    def x_h_correct(self,data):
        
        x = np.zeros([9,1])

        self.z = np.vstack((x,self.u_delta_h)) 

        
        # This is done like that so the dimensions fits together self.x_h.shape=(9,1) normally
        x_h_dimMatch= np.vstack((self.x_h,np.zeros([5,1])))
        
        # z = z + K*(sensorData - H*x_h)
        self.z = self.z + np.dot(self.K,(self.sensorData - np.dot(self.H,x_h_dimMatch)))


        delta_angle = self.z[6:9]
        self.add_pertubians(delta_angle)  
        self.update_u_delta_h() # Remember the bias terms

    def add_pertubians(self,delta_angle):

        # position + speed
        self.x_h[0:6] = self.x_h[0:6] + self.z[0:6]
        
        # Oriantation - This is from Isaacs formulations
        qtemp = Quaternion(w=self.x_h[6],x=self.x_h[7],y=self.x_h[8],z=self.x_h[9])
        qRot = qtemp.get_rot() 
        OMEGA = self.makeSkew(delta_angle)
        
        # correct the oriantation with small angels
        # R = (I_3 - OMEGA)*R
        R = np.dot((np.eye(3)-OMEGA),qRot)

        # Convert it rot matrix back to quaternions
        qtemp = qtemp.rot_to_quat(R)._q

        self.x_h[6:10] = qtemp

    def update_u_delta_h(self):
        self.u_delta_h = self.z[9:15]

    def cal_P_correct(self):
        
        I = np.eye(15)
        # P = (I-KH)*P
        self.P = np.dot(np.dot((I-self.K),self.H),self.P)

    def cal_P_predict(self,F):
        # P = F*P*F.T + Q*dt

        #self.P = F*self.P*F.T+self.Q*self.dt
        self.P = np.dot(np.dot(F,self.P),F.T)+self.Q*self.dt

    def cal_posVel_predict(self,acc_raw,quat):

        
        #self.P_num = self.P_num  + self.dt*self.V_num + self.dt^2 *0.5(self.quat_num.get_rot()*acc_raw-self.acc_bias-self.g)
        #self.V_num = self.V_num + self.dt*(self.quat_num.get_rot()*acc_raw-self.acc_bias-self.g)

        # Transform it to global reference
        u = quat.get_rot().dot(acc_raw)

        # Subtrackt gravity
        u = u + self.g    

        # State-space model 
        A = np.eye(6)
        A[0,3] = self.dt
        A[1,4] = self.dt
        A[2,5] = self.dt
        Bp = np.array((self.dt**2.0)/2.0*np.eye(3))
        Bv = np.array(self.dt*np.eye(3))
        B = np.vstack((Bp,Bv))

        # Position and velocity
        self.x_h[0:6] = np.dot(A,self.x_h[0:6]) + B.dot(u)

    def cal_quat_predict(self,gyro_raw):
        
        w_norm = np.linalg.norm(gyro_raw)
        I=np.eye(4)
        OMEGA = self.get_OMEGA(gyro_raw)

        if(w_norm != 0.0):
            self.x_h[6:10] = np.dot((cos(0.5*self.dt*w_norm)*I + (1/w_norm)*sin(0.5*self.dt*w_norm)*OMEGA)
            ,self.x_h[6:10])

    def get_OMEGA(self,gyro_raw):
        #This is right handed oriantation
        x = gyro_raw[0]
        y = gyro_raw[1]
        z = gyro_raw[2]

        OMEGA = np.zeros((4,4))

        OMEGA[0,0:4] = [0,-x,-y,z]
        OMEGA[1,0:4] = [x,0,z,-y]
        OMEGA[2,0:4] = [y,-z,0,x]
        OMEGA[3,0:4] = [z,y,-x,0]

        return OMEGA

    def makeSkew(self,vec):

        M = np.zeros((3,3))

        M[1,0] = -vec[2]
        M[2,0] = vec[1]

        M[0,1] = vec[2]
        M[2,1] = -vec[0]

        M[0,2] = -vec[1]
        M[1,2] = vec[0]

        return M

    # The Noise matrix isn't included
    def Fmatrix_update(self,acc,quat):

        Qrot = quat.get_rot()
        rotCrossMat = self.makeSkew(np.dot(Qrot,acc))

        I = np.eye(3)
        O = np.zeros((3,3))
        
        # The timestep will be multipled at the end
        l1 = np.hstack((O, I, O, O, O ))
        l2 = np.hstack((O, O, -rotCrossMat, Qrot, O))
        l3 = np.hstack((O, O, O, O, -Qrot))
        l4 = np.hstack((O, O, O, O, O))
        l5 = np.hstack((O, O, O, O, O))
        
        F = np.vstack((l1,l2,l3,l4,l5))

        F = np.eye(15)+F*self.dt

        return F


if __name__ == '__main__':
    try:
        ES_EKF()
    except rospy.ROSInterruptException as e:
        print (e)


