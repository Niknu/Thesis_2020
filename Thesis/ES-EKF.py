#!/usr/bin/env python3

import numpy as np
from scipy.stats import chi2
from math import *
import rospy
from message_filters import *  # For subscribe to multiple topics and do one thing

from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension


from mavros_msgs.msg import Altitude
from sensor_msgs.msg import NavSatFix, Imu, MagneticField    #From the /mavros/global_position/global or mavros/global_position/raw/fix
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates # For the quaternions
from std_msgs.msg import Bool

from ecef2geodtic_def import * # This is for running the function GEOToNED
#from madgwickahrs import MadgwickAHRS
from quaternion import Quaternion 


import csv

import os


'''
TODO:

'''




class ES_EKF():
    def __init__(self):

        #########
        # Debug #
        #########
        self.delta_angle_debug = np.zeros((3,1))
        self.quat_global_debug = np.zeros((4,1))
        self.clear = lambda: os.system('clear')

        self.IMU_ready = True
        self.GPS_ready = True

        ###################################
        # Setting for writing to csv-file 
        ###################################

        self.write_csv = True
        self.logGT = True
        self.spoofOn = False # normal bool value 
        
        self.spoofMethod = "Measurement"+"_"  #Measurement or Innovation (Cov_matrix)
        self.fileName = 'SITL_GNSS_4point5rounds' #'SITL_OptiTrack_straightLine_spoofing_01'#'OptiTrack_grond2squareValidate_KF_5rounds' #'AtoB_11_with_quat_rot_global' # ground_spoofing

        self.kalman_validation = True
        self.kalman_test_name = 'GPS_Qdefault_'+ self.fileName

        self.fileName += '.csv'

        ########################################
        # De/activation of Kalman filter parts #
        ########################################

        self.add_quaternion_pertubians = False
        self.active_resetP = False


        ###############################
        # Spoofing detection theshold #
        ###############################

        alpha = 0.0001 # NOTE: This is chosen because we to say when it's 0.01%(EXSTREM case) out of the distribution
        DOF = 2 # NOTE: This is 2 because we only measure x,y position (not z yet)
        self.threshold = chi2.ppf(1-alpha,df=DOF) 
        
        ##############################################
        # Ground truth data from Gazebo or OptiTrack #
        ##############################################

        self.gt_pos = np.zeros((3,1))   # The ground Truth from Gazebo
        self.gt_pos_0 = np.zeros((3,1))
        self.gt_start = True
        self.gt_vel = np.zeros((3,1))
        self.gt_quat = np.array((1,0,0,0),dtype=np.float64).reshape((4,-1))
        self.gt_spoofing_active = False


        ##################
        #   OptiTrack    #
        ##################
        self.optiTrack_real = False
        self.optiTrack = False  # True when optiTrack used - sets Q and R different depending on  + TODO: add or remove z-pos estimation(sets to 0)
        self.setZaxisTo0 = True # sets all the variables with z = 0
        self.optiTrack_pos_0 = np.zeros([3,1])
        self.optiTrack_pos_first = True


        ###################
        # For time update #
        ###################
        self.t_current = None
        self.t_prev = None
        self.FirstRun_dt = True
        self.dt = 0.02 # This is updated by a function
        self.time = 0.0 # [s]

        #############################
        # Kalman Filter init params #
        #############################

        # The value from: https://docs.px4.io/master/en/advanced_config/parameter_reference.html#EKF2_EVV_NOISE
        
        # The range for the variance for the mean drift and data noise
        #                               min      default          max
        #acc_bias_var    |                0          3/1000        0.01  # bias noise [m/s^3]
        #gyro_bias_var   |                0          1/1000        0.01  # bias noise [rad/s^2]
        #acc_meas_noise  |                0.01       3.5/10        1.0   # measurement noise [m/s^2]
        #gyro_meas_noise |                0.0001     1.5/100       0.1   # measurement noise [rad/s]

        acc_bias_var = 3/1000#              # bias noise [m/s^3]
        gyro_bias_var = 1/1000#             # bias noise [rad/s^2]
        acc_meas_noise = 3.5/10#             # measurement noise [m/s^2]
        gyro_meas_noise = 1.5/100#           # measurement noise [rad/s]

        ###############################------TODO-----Change something here---------------###################################################
        if self.optiTrack == True:
            pos_var = 0.1           # Measurement noise (vision) [m] (EKF2_EVP_NOISE)
        else:
            pos_var = 0.5           # Measurement noise (GPS) [m]


        no_states = 15  # Number of error states       TODO: Could be 14 due to z axis could be removed due to the height can't be estimated with the IMU
        no_measure = 3  # Number of measureable states TODO: Could be 2 because of the height(z-axis) can be measured with the GPS
        self.sensorData = np.zeros([no_measure,1])              # It will contain the NED order where N=y, E=x , D=(z-downwards) so [0]=N,[1]=E,[2]=D
        
        
        self.g = np.array([[0,0,-9.8066]]).reshape((3,-1)) # prev estimatio = -9.803868 --- Gazebo value = -9.8066
        self.acc_data = np.zeros([3,1])                         # It will contain the NED order where N=y, E=x , D=(z-downwards) so [0]=N,[1]=E,[2]=D
        self.gyro_data = np.zeros([3,1])                        # It will contain the NED order where N=y, E=x , D=(z-downwards) so [0]=N,[1]=E,[2]=D
        self.acc_raw = np.zeros([3,1])
        self.gyro_raw = np.zeros([3,1])
        self.gyro_prev = np.zeros([3,1])

        self.P = np.eye(no_states)*10.0#*100.0 #TODO: try with *10 or *100  instead                             # Covariance Matrix for KF
        self.Q = np.zeros((no_states,no_states))                       # Unsernety for the prediction, This contains measurement noise and bias noise due to model the 'random walk' behavior, - Indipendent! therefore dim=no_states
        
        self.u_delta_h = np.zeros((6,1))                        # The bias update for the sensors
        self.u = np.zeros((6,1))                                # The vector with the sensor input + bias
        self.x_h = np.zeros((10,1))                             # State vector
        
        if self.optiTrack_real == True:
            self.x_h[6:10] = Quaternion(w_or_q=-1,x=0,y=0,z=0)._q
        else:
            self.x_h[6:10] = Quaternion(w_or_q=1,x=0,y=0,z=0)._q

        self.z = np.zeros((no_states,1))                               # Vector holing the error of the state and the bias from the sensors
        self.H = np.zeros((no_measure,no_states))                              # Only used for the GPS signal
        self.F = None # More details, look at the function 'Fmatrix_update()' It's generated there
        self.R = np.eye(no_measure)                             # <.-- The GPS only or the quat also(madwick filter) ?   # Unsernety for the sensors     #At this current time it's only the GPS data

        if self.setZaxisTo0 == True:

            self.H[0,0] = 1     #y-pos
            self.H[1,1] = 1     #x-pos
            self.H[2,2] = 0     #z-pos

            self.R[2,2] = 0     #z-pos
            self.P[2,2] = 0     #z-pos_cov
            self.P[5,5] = 0     #z-vel_cov
            self.g = np.array([[0,0,0]]).reshape((3,-1))

            #measurement noise
            #self.Q[3:6,3:6] = np.eye(3)*acc_meas_noise
            self.Q[3:5,3:5] = np.eye(2)*acc_meas_noise # This is only 2 states because all z-axis for pos+vel  is = 0
            self.Q[6:9,6:9] = np.eye(3)*gyro_meas_noise

            # bias noise
            #self.Q[9:12,9:12] = np.eye(3)*acc_bias_var
            self.Q[9:11,9:11] = np.eye(2)*acc_bias_var # This is only 2 states because all z-axis for pos+vel  is = 0
            self.Q[12:15,12:15] = np.eye(3)*gyro_bias_var

        else:

            self.H[0,0] = 1     #y-pos
            self.H[1,1] = 1     #x-pos
            self.H[2,2] = 1     #z-pos

            #measurement noise
            self.Q[3:6,3:6] = np.eye(3)*acc_meas_noise
            self.Q[6:9,6:9] = np.eye(3)*gyro_meas_noise

            # bias noise
            self.Q[9:12,9:12] = np.eye(3)*acc_bias_var
            self.Q[12:15,12:15] = np.eye(3)*gyro_bias_var

        self.R *= pos_var
        self.Q[0:3,0:3] = 0 # position doesn't have noise

        # Square the values to be std.dev instead of variance
        self.R *=2
        self.Q *=2

        #for x in range(0,no_states):
        #    print(self.Q[x,:])
        #print(self.Q.shape)
        #exit()

        #######################
        # GPS transform setup #
        #######################
        self.GPS_start = True
        self.GPS_0 = np.zeros([3,1])
        self.NED = np.zeros([3,1])
        self.rot_GPS = np.zeros([3,3]) # The output from the function in rotMat_NED or ENU
        

        

        ######################
        # Spoofing detection #
        ######################
        # For the measurement avg approach
        self.window_size = 5 # chosen randomly 
        self.z_avg = np.zeros((self.window_size,no_measure,1)) # This is for GPS position NOTE: It's 3D array
        self.z_i = 0 # The indexing for the z_avg --> z_avg[z_i , :]
        self.x_avg = np.zeros((self.window_size,no_measure,1)) # NOTE: It's 3D array
        self.x_i = 0

        # For the cov-matrix avg approach
        self.P_inv_gamma_array = np.zeros((self.window_size,no_measure,no_measure))
        self.P_i = 0
        self.gamma_array = np.zeros((self.window_size,no_measure,1))
        self.qChi = 0
        self.qChi_meas = 0
        self.qChi_innv = 0
        
        '''
        # --- Madgwick filter --- #
        #self.beta = 0.334 # This is from his own paper where he point out where it's optimal
        #self.madFilt=MadgwickAHRS(beta=self.beta,sampleperiod=1/self.dt)
        '''

        #############
        # ROS setup #
        #############

        #init the node and subscribe to the GPS adn IMU
        rospy.init_node('ES_EKF',anonymous=True)

        if self.optiTrack == True:
            rospy.Subscriber('/mavros/vision_pose/pose',PoseStamped, self.correct_KF_GPS)
            #rospy.Subscriber('/mavros/local_position/pose',PoseStamped, self.correct_KF_GPS)
            if self.optiTrack_real == True:
                rospy.Subscriber('/vrpn_client_node/drone/pose', PoseStamped, self.groundTruth)
            else:
                rospy.Subscriber('/gazebo/model_states', ModelStates, self.groundTruth)
        else:
            rospy.Subscriber('/mavros/global_position/raw/fix',NavSatFix, self.correct_KF_GPS)
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.groundTruth)

        #rospy.Subscriber('/mavros/altitude',Altitude, self.update_z) # This isn't used
        rospy.Subscriber('/mavros/imu/data_raw',Imu, self.prediction_KF_IMU)
        rospy.Subscriber('/spoofing_active_signal',Bool,self.reading_active_spoofing)

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

        acc[0] =imuData.linear_acceleration.y
        acc[1] =imuData.linear_acceleration.x
        acc[2] =imuData.linear_acceleration.z

        gyro[0] =imuData.angular_velocity.y
        gyro[1] =imuData.angular_velocity.x
        gyro[2] =imuData.angular_velocity.z

        mag[0] = magData.magnetic_field.y
        mag[0] = magData.magnetic_field.x
        mag[0] = magData.magnetic_field.z

        __ , self.qDot = self.madFilt.update(gyroscope=gyro,accelerometer=acc,magnetometer=mag) # the output is the quat(not used) and qdot

        print(self.qDot[0],self.qDot[1],self.qDot[2],self.qDot[3])
    '''

    def reading_active_spoofing(self,data):

        self.gt_spoofing_active = data.data 

    def update_dt(self,data):
        t_secs_new = data.header.stamp.secs
        t_nsecs_new = data.header.stamp.nsecs*1.0e-9            # Convert from nsecs to secs

        t_current = t_secs_new + t_nsecs_new
        
        if self.FirstRun_dt is not True:
            self.dt = t_current - self.t_prev                   #Find the time difference (delta t)
            self.time += self.dt
        self.t_prev = t_current

        self.FirstRun_dt = False

        return self.dt

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
            self.GPS_start =  False

        self.rotMat_NED(lat,lon)

        self.NED = self.rot_GPS @ (XYZ-self.GPS_0)

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
            self.GPS_start =  False

        #self.rotMat_NED(lat,lon) # Have changed the sign in for the z part in R!!!
        self.rotMat_NED(lat,lon)

        # NED = rot_GPS*(XYZ-GPS_0)
        self.NED = self.rot_GPS @ (XYZ-self.GPS_0)    

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

    def init_multdata_NEW(self,data):
        msg = MultiArrayLayout()

        msg.data_offset = 0

        msg.dim = [MultiArrayDimension()]

        msg.dim[0].label= "state_estimation"
        msg.dim[0].size = data.shape[0]
        msg.dim[0].stride = data.shape[0]

        return msg

    #NOTE: Maybe the height for the should be injected at the end of the GeotoNED function
    def update_z(self,data):
        self.GPS_z = data.amsl  # Above Mean Sea Level

    def prediction_KF_IMU(self,data_IMU):

        #if self.IMU_ready == True:
        self.IMU_ready = False
        dt = self.update_dt(data_IMU)

        # Load data and correct data with the bias
        self.acc_data[0] = data_IMU.linear_acceleration.x - self.u_delta_h[0]
        self.acc_data[1] = data_IMU.linear_acceleration.y - self.u_delta_h[1]
        self.acc_raw[0] = data_IMU.linear_acceleration.x
        self.acc_raw[1] = data_IMU.linear_acceleration.y
        
        if self.setZaxisTo0 == True:
            self.acc_data[2] = 0
            self.acc_raw[2] = 0
        else:
            self.acc_data[2] = data_IMU.linear_acceleration.z - self.u_delta_h[2]
            self.acc_raw[2] = data_IMU.linear_acceleration.z

        # Corrected
                                                        # The bias term is cancelled out(+-) because of the estimation of the bias is too big compared to what it really is. 
        self.gyro_data[0] = data_IMU.angular_velocity.x - self.u_delta_h[3] + self.u_delta_h[3]
        self.gyro_data[1] = data_IMU.angular_velocity.y - self.u_delta_h[4] + self.u_delta_h[4]
        self.gyro_data[2] = data_IMU.angular_velocity.z - self.u_delta_h[5] + self.u_delta_h[5]
        
        # Raw
        self.gyro_raw[0] = data_IMU.angular_velocity.x
        self.gyro_raw[1] = data_IMU.angular_velocity.y
        self.gyro_raw[2] = data_IMU.angular_velocity.z

        '''
        wq = data_IMU.orientation.w
        xq = data_IMU.orientation.x
        yq = data_IMU.orientation.y
        zq = data_IMU.orientation.z
        self.x_h[6] = wq
        self.x_h[7] = xq
        self.x_h[8] = yq
        self.x_h[9] = zq
        '''
        quat_temp = Quaternion(w_or_q=self.x_h[6],x=self.x_h[7],y=self.x_h[8],z=self.x_h[9])

        

        # Predict x_h
        self.cal_posVel_predict(self.acc_data,quat_temp,dt)                    #########################################
        self.cal_quat_predict(self.gyro_data,dt)              #<<------------- TODO:NEEDS to be enabled when it's fixed ------------------
                                                                            #########################################


        F = self.Fmatrix_update(self.acc_data,quat_temp,dt)

        self.cal_P_predict(F,dt)

        wp=self.x_h[6]
        xp=self.x_h[7]
        yp=self.x_h[8]
        zp=self.x_h[9]
        #xroll = atan2(2*(wp*xp+yp*zp),(1-2*(yp**2+zp**2)))
        #ypitch = asin(2*(wp*yp-zp*xp))
        #zyaw = atan2(2*(wp*zp+xp*yp),(1-2*(yp**2+zp**2)))
        
        np.set_printoptions(suppress=True,precision=8)

        
        self.clear()
        print('--GPS position --')
        print(self.sensorData)
        print('--KF position  --')
        print(self.x_h[0:3])
        print('-- vel --')
        print(self.x_h[3:6])
        print('-- Delta Angle DEBUG * 0.5 --')
        print(self.delta_angle_debug * 0.5)
        print('--quat--')
        print(self.x_h[6:10])
        #print('-- quat Rot Matrix --')
        #print(qprint.get_rot())
        print('in euler deg:')
        #print('x/roll = ',xroll*(180/pi))
        #print('y/pitch = ',ypitch*(180/pi))
        #print('z/yaw = ',zyaw*(180/pi))
        #print('--chi--+method',self.spoofMethod)
        print('chi_innv',self.qChi_innv)
        print('chi_meas',self.qChi_meas)
        print('acc_raw')
        print(self.acc_raw)
        print('acc_raw after rotaion')
        print(quat_temp.get_rot() @ self.acc_raw )
        print('gyro_raw')
        print(self.gyro_raw)
        print('-- delta_u_hat--')
        print(self.u_delta_h)
        print('-- Time --')
        print(self.time)
        
        
        self.write_to_csv_file_kalmanValidation('Prediction',dt)

        self.gyro_prev = self.gyro_data
        self.IMU_ready = True

    def correct_KF_GPS(self,data_GPS):
        #if self.GPS_ready == True:
        self.GPS_ready = False 
        dt = self.update_dt(data_GPS)

        #print('-- GPS-- dt')
        #print(dt)

        if (self.optiTrack == True and self.optiTrack_real == False):

            if(self.optiTrack_pos_first==True):
                self.optiTrack_pos_0[0] = data_GPS.pose.position.x
                self.optiTrack_pos_0[1] = data_GPS.pose.position.y
                
                self.sensorData[0] = 0
                self.sensorData[1] = 0
                self.sensorData[2] = 0

                self.optiTrack_pos_first = False
            else:
                self.sensorData[0] = data_GPS.pose.position.x
                self.sensorData[1] = data_GPS.pose.position.y
                self.sensorData[2] = 0#data_GPS.pose.position.z

                # Correct for what is zero
                #self.sensorData -= self.optiTrack_pos_0
        elif (self.optiTrack_real == True):
            if(self.optiTrack_pos_first==True):
                self.optiTrack_pos_0[0] = data_GPS.pose.position.y
                self.optiTrack_pos_0[1] = data_GPS.pose.position.x
                
                self.sensorData[0] = 0
                self.sensorData[1] = 0
                self.sensorData[2] = 0

                self.optiTrack_pos_first = False
            else:
                self.sensorData[0] = data_GPS.pose.position.y
                self.sensorData[1] = data_GPS.pose.position.x
                self.sensorData[2] = 0#data_GPS.pose.position.z


        else:
            self.GeoToNED_old(data_GPS.latitude,data_GPS.longitude,data_GPS.altitude)
            self.sensorData[0] = self.NED[1]    #x
            self.sensorData[1] = self.NED[0]    #y
            self.sensorData[2] = 0#self.NED[2]    #z (z are pointing downwards)


        self.cal_KF_gain()
        self.x_h_correct()
        self.cal_P_correct()

        # Is it need with P reset as mention in the literature?
        
        if self.active_resetP == True:
            delta_angle=self.z[6:9]
            self.resetP(delta_angle)
        else:
            # resetP shouldn't be used if angle delta_angle is too big. This reason behind the delta_angle gets too big is because the kalman gain, that is set by the cov matrix self.P
            pass
        
        # If it's need decide which method should be used
        '''
        if self.spoofMethod == ("Innovation_"):
            self.qChi = self.chi_cal_innovation_method() 
        if self.spoofMethod == ("Measurement_"):
            self.qChi = self.chi_cal_measurement_method()
        '''
        self.qChi_innv = self.chi_cal_innovation_method() 
        
        self.qChi_meas = self.chi_cal_measurement_method()
        
        
        #print('============'+self.spoofMethod)
        #print('chi =', self.qChi[0,0])

        self.pub_xhat_data_NEW(self.x_h)

        self.write_to_csv_file_spoofing()
        self.write_to_csv_file_kalmanValidation('Correction',dt)
        self.write_to_csv_file_POS_KF_GPS_GT()
        
        self.GPS_ready = True

    def x_h_correct(self):
        
        x = np.zeros([9,1])

        if self.setZaxisTo0 == True:
            self.u_delta_h[2] = 0   # z-acceleration

        self.z = np.vstack((x,self.u_delta_h)) 

        # This is done like that so the dimensions fits together self.x_h.shape=(10,1) normally
        x_h_dimMatch = np.vstack((self.x_h,np.zeros([5,1])))
        
        #print('---- sensorData ----')
        #print('pos-z',self.sensorData[2])
        #print('vel-z',self.sensorData[5])

        #print('-- z before correction --')
        #print(self.z)


        # z = z + K*(sensorData - H*x_h)
        self.z = self.z + self.K @ (self.sensorData - (self.H @ x_h_dimMatch))
        


        if self.setZaxisTo0 == True:
            self.z[2] = 0   # z-pos
            self.z[5] = 0   # z-vel
            self.z[11] = 0  #z-acc_bias noise

        '''
        print('K=')
        print(self.K)
        print()
        print('y =')
        print(self.sensorData)
        print('H=')
        print(self.H)
        print('x_h_dim_match=')
        print(x_h_dimMatch)
        print('z=')
        print(self.z)
        #print()
        #print('P=')
        #print(self.P)
        print('-- Time --')
        print(self.time)
        #print('-- z after correction --')
        #print(self.z)
        #print('-- z[9:15]--')
        #print(self.z[9:15])
        '''


        delta_angle = self.z[6:9]
        self.delta_angle_debug = delta_angle
        self.add_pertubians(delta_angle)  
        #Updating u_delta_h:
        self.u_delta_h = self.z[9:15]

    def add_pertubians(self,delta_angle):


        # position + speed
        self.x_h[0:6] = self.x_h[0:6] + self.z[0:6]
        
        #Set the pos and speed to 0
        if self.setZaxisTo0 == True:
            self.x_h[2,0] = 0  #pos-z
            self.x_h[5,0] = 0  #vel-z


        # - This is form Joan Sola --------- TODO: Is delta_angle an angle or is it angle speed? else q1 = angle_speed*sin(angle/2)
        if self.add_quaternion_pertubians == True:
            q_current = Quaternion(w_or_q=self.x_h[6],x=self.x_h[7],y=self.x_h[8],z=self.x_h[9])
            q_error = Quaternion(w_or_q=1, x=delta_angle[0]*0.5, y=delta_angle[1]*0.5, z=delta_angle[2]*0.5)
            # Correct quaternion with global error correction
                # ---------- Decide if norm is needed? -----------
            self.x_h[6:10] = (q_error * q_current).norm_quat()._q # Global error and output it as unit quaternion
            #self.x_h[6:10] = (q_error * q_current)._q  # Global error
            #self.x_h[6:10] = (q_current * q_error)._q  # Local error


        # End of Joan Sola formulation

    def cal_KF_gain(self):

        #ERROR after some time when P is set to reset
        # LinAlgError: SVD did not converge

        # NOTE: Reason to error could be - (1) RAM issue -  (2) The matrix contains NaN or Inf
        # NOTE: From test it looks like it was a RAM issue due to previuse experience this happend when Gazebo gui was running also (The PC 8GB of RAM) 

        # K = P*H.T*inv(H*P*H.T+R)
        s = self.H @ self.P @ self.H.T + self.R 
        S = np.linalg.pinv(s)
        self.K = self.P @ self.H.T @ S

    def cal_P_correct(self):
        
        I = np.eye(15)
        #P = (I-KH)*P <-- just how the formula is 
        self.P = (I-self.K @ self.H) @ self.P

        # Attemp for remove the error(SVD did not converge) in the function cal_KF_gain
        #symmetric and positive Joseph form is pick (p.63 bottom - Joan Sola)
        #self.P =  (I-self.K @ self.H) @ self.P @ (I-self.K @ self.H).T + self.K @ self.R @ self.K.T

        #The symmetric form 
        
        #self.P = self.P - self.K @ (self.H@self.P@self.H.T + self.R) @ self.K.T

        if self.setZaxisTo0 == True:
            self.P[2,2] = 0 #pos-z
            self.P[5,5] = 0 #vel-z

    def cal_P_predict(self,F,dt):

        self.P = F @ self.P @ F.T + self.Q * dt

        if self.setZaxisTo0 == True:
            self.P[2,2] = 0 #pos-z
            self.P[5,5] = 0 #vel-z

    def cal_posVel_predict(self,acc_raw,quat,dt):

        # Transform it to global reference
        R = quat.get_rot()
        u_before = R @ (acc_raw)  

        


        # Subtrackt gravity
        u = u_before + self.g

        if self.setZaxisTo0 == True:
            u[2] = 0    # z - setting to zero

        '''
        print('-- accelration before rotation --')
        print(u_before)
        print('-- accelration after rotation --')
        print(u)
        print('---Rot_quat---')
        print(R)
        '''

        # State-space model 
        A = np.eye(6)
        #A[0,3] = self.dt    # x
        #A[1,4] = self.dt    # y
        
        # NOTE: Without self.dt
        A[0,3] = dt
        A[1,4] = dt


        #Bp = np.array((self.dt**2.0)/2.0*np.eye(3))
        #Bv = np.array(self.dt*np.eye(3))

        # NOTE: Without self.dt
        Bp = np.array((dt**2.0)/2.0*np.eye(3))
        Bv = np.array(dt*np.eye(3))

        if self.setZaxisTo0 == True:
            A[2,5] =  0 # state - pos-z
            Bp[2,2] = 0 # input - pos-z
            Bv[2,2] = 0 # input - vel-z
        else:
            #A[2,5] = self.dt        # z
            A[2,5] = dt # state - pos-z

        B = np.vstack((Bp,Bv))


        # Position and velocity
        self.x_h[0:6] = A @ self.x_h[0:6] + B @ u

        # Sets pos+vel in the z-axis to 0 because it needs more work to be able to estimate the height
        if self.setZaxisTo0 == True:
            self.x_h[2] = 0    #pos-z
            self.x_h[5] = 0    #vel-z

    def cal_quat_predict(self,gyro_raw,dt):
        
        #gyro_raw[2] = 0 # z - setting to 0

        #xg = gyro_raw[0]
        #yg = gyro_raw[1]
        #zg = gyro_raw[2]

        # If newton method doesn't work look https://arxiv.org/pdf/1711.02508.pdf at p. 87
        
        #qlocal_prev_rot = Quaternion(w_or_q=self.x_h[6],x=self.x_h[7],y=self.x_h[8],z=self.x_h[9]).get_rot()

        

        '''
        avg_gyro = gyro_raw #(gyro_raw + self.gyro_prev)/2

        gyro_global = qlocal_prev_rot @ avg_gyro

        OMEGA = self.get_OMEGA(gyro_global)

        q_dot = 0.5*OMEGA @ self.x_h[6:10]

        #q_next = self.x_h[6:10] + q_dot*dt

        q_next = self.x_h[6:10] + q_dot*dt

        q_norm = np.linalg.norm(q_next)
        if q_norm > 1:
            q_next /= q_norm 

        self.x_h[6:10] = q_next
        '''

        
        #print('Qrot----------')
        #print(self.quat_global_debug)

        # Zero-order integration method
        avg_gyro = gyro_raw #(gyro_raw + self.gyro_prev)/2

        #TODO:
        w_norm = np.linalg.norm(avg_gyro)*dt
        I=np.eye(4)
        OMEGA = self.get_OMEGA(avg_gyro*dt)#*0.5

        #print('omega norm= ',w_norm)

        if(w_norm >= 1e-8):#!= 0.0): # >=1e-8 instead of !=0.0?
            self.x_h[6:10] = (cos(w_norm/2)*I + (1/w_norm)*sin(w_norm/2)*OMEGA) @ self.x_h[6:10]
        

        '''
            w = cos(w_norm*self.dt/2)
            x = sin(w_norm*self.dt/2)*xg/w_norm
            y = sin(w_norm*self.dt/2)*yg/w_norm
            z = sin(w_norm*self.dt/2)*zg/w_norm

            qtemp = Quaternion(w_or_q=w,x=x,y=y,z=y)
            qprev = Quaternion(w_or_q=self.x_h[6],x=self.x_h[7],y=self.x_h[8],z=self.x_h[9])
            
            qest = qtemp * qprev

            self.x_h[6] = qest._q[0]    # w
            self.x_h[7] = qest._q[1]    # x
            self.x_h[8] = qest._q[2]    # y
            self.x_h[9] = qest._q[3]    # z
        '''
        '''
        #For debugging with the ground truth (topic /gazebo/model_states)
        tmp_c = "iris"
        for x in range(0,len(gyro_raw.name)):
            if (gyro_raw.name[x] == tmp_c):
                
                #print('x=',x)
                #print(gyro_raw.pose[x].orientation)

                qx = gyro_raw.pose[x].orientation.x
                qy = gyro_raw.pose[x].orientation.y
                qz = gyro_raw.pose[x].orientation.z
                qw = gyro_raw.pose[x].orientation.w

                # For debugging 
                self.x_h[6:10] =np.array((qw,qx,qy,qz)).reshape(4,-1) 
        '''

    def get_OMEGA(self,gyro_raw):
        
        x = gyro_raw[0]
        y = gyro_raw[1]
        z = gyro_raw[2]

        OMEGA = np.zeros((4,4))
        
        # make the gyro vector to a skew matrix
        OMEGA[0,:] = [0,-x,-y,-z]
        OMEGA[1,:] = [x,0,-z,y]
        OMEGA[2,:] = [y,z,0,-x]
        OMEGA[3,:] = [z,-y,x,0]
        

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

    def Fmatrix_update(self,acc,quat,dt):

        Qrot = quat.get_rot()
        rotCrossMat = self.makeSkew(Qrot @ acc)

        I = np.eye(3)
        O = np.zeros((3,3))
        
        # The timestep will be multipled at the end
        l1 = np.hstack((O, I, O, O, O ))
        l2 = np.hstack((O, O, -rotCrossMat, -Qrot, O))
        l3 = np.hstack((O, O, O, O, -Qrot))
        l4 = np.hstack((O, O, O, O, O))
        l5 = np.hstack((O, O, O, O, O))
        
        F = np.vstack((l1,l2,l3,l4,l5))

        #F = np.eye(15)+F*self.dt
        F = np.eye(15)+F*dt

        if self.setZaxisTo0 == True:
            F[2,2] = 0 #pos-z
            F[5,5] = 0 #vel-z

        return F

    def write_to_csv_file_spoofing(self):
        if self.write_csv == True:
            
            #self.gt_spoofing_active
            
            if(self.spoofOn==True):
                #file_chi = open('Sp_ON_'+self.spoofMethod+'Innovation_'+self.fileName,'a')
                file_chi = open('Sp_ON_' + self.fileName,'a')
                
                file_chi.write(str(self.qChi_meas[0,0]) + ',')
                file_chi.write(str(self.qChi_innv[0,0]) + ',')
                file_chi.write(str(self.threshold) + ',')
                file_chi.write(str(self.time) + ',')
                file_chi.write(str(self.gt_spoofing_active))
                file_chi.write("\n")
                file_chi.close()

            elif (self.spoofOn==False):
                #file_chi = open('Sp_OFF_'+self.spoofMethod+self.fileName,'a')
                file_chi = open('Sp_OFF_' + self.fileName,'a')

                file_chi.write(str(self.qChi_meas[0,0]) + ',')
                file_chi.write(str(self.qChi_innv[0,0]) + ',')
                file_chi.write(str(self.threshold) + ',')
                file_chi.write(str(self.time)+',')
                file_chi.write(str(self.gt_spoofing_active))
                file_chi.write("\n")
                file_chi.close()

    def write_to_csv_file_kalmanValidation(self,name_preOrCor,dt):
        if self.write_csv == True:
            if(self.kalman_validation==True):
                file_KF = open ('KalmanValidation_'+self.kalman_test_name +'_'+ name_preOrCor + '.csv','a')
                
                #pos_KF_xyz(0:2)
                file_KF.write(str(self.x_h[0,0])+ "," + str(self.x_h[1,0])+ "," + str(self.x_h[2,0])+ ",") 
                #pos_GPS_xyz(3:5)
                file_KF.write(str(self.sensorData[0,0])+ "," + str(self.sensorData[1,0])+ "," + str(self.sensorData[2,0])+ ",") 
                #vel_xyz(6:8)
                file_KF.write(str(self.x_h[3,0])+ "," + str(self.x_h[4,0])+ "," + str(self.x_h[5,0])+ ",")
                #acc_bias_xyz(9:11)
                file_KF.write(str(self.u_delta_h[0,0])+ "," + str(self.u_delta_h[1,0])+ "," + str(self.u_delta_h[2,0])+ ",")
                #gyro_bias_xyz(12:14)
                file_KF.write(str(self.u_delta_h[3,0])+ "," + str(self.u_delta_h[4,0])+ "," + str(self.u_delta_h[5,0])+ ",")
                #raw_acc_xyz(15:17)
                file_KF.write(str(self.acc_raw[0,0])+ "," + str(self.acc_raw[1,0])+ ","+str(self.acc_raw[2,0])+ "," )
                #raw_gyro_xyz(18:20)
                file_KF.write(str(self.gyro_raw[0,0])+ "," + str(self.gyro_raw[1,0])+ "," + str(self.gyro_raw[2,0])+",")
                #quat_wxyz(21:24)
                file_KF.write(str(self.x_h[6,0])+ "," +str(self.x_h[7,0])+ "," +str(self.x_h[8,0])+ "," +str(self.x_h[9,0])+",")
                # Cov matrix_pos_xyz (25:27)
                file_KF.write(str(self.P[0,0])+ "," + str(self.P[1,1])+ "," + str(self.P[2,2])+",")
                # Cov matrix_vel_xyz (28:30)
                file_KF.write(str(self.P[3,3])+ "," + str(self.P[4,4])+ "," + str(self.P[5,5])+",")
                # Cov matrix_angle_xyz (31:33)
                file_KF.write(str(self.P[6,6])+ "," + str(self.P[7,7])+ "," + str(self.P[8,8])+",")
                # Cov matrix_acc_bias_xyz (34:36)
                file_KF.write(str(self.P[9,9])+ "," + str(self.P[10,10])+ "," + str(self.P[11,11])+",")
                # Cov matrix_gyro_bias_xyz (37:39)
                file_KF.write(str(self.P[12,12])+ "," + str(self.P[13,13])+ "," + str(self.P[14,14])+",")
                #delta_angle_xyz (40:42)
                file_KF.write(str(self.z[6,0]) + ','+ str(self.z[7,0]) + ',' + str(self.z[8,0]) + ',')
                #Time in seconds(43)
                file_KF.write(str(self.time) + ',')
                file_KF.write("\n")
                file_KF.close()

    def write_to_csv_file_POS_KF_GPS_GT(self):
        # Data to plot the position for the drone against KF+GPS+Ground Truth
        if self.write_csv == True:
            if(self.kalman_validation==True):
                file_gt = open('Kalman_posVsGPSVsGroundTruth_'+self.fileName ,'a')

                #Pos_KF_xy (0:1)
                file_gt.write(str(self.x_h[0,0])+ "," + str(self.x_h[1,0])+ "," )
                #Pos_GPS_xy(2:3)
                file_gt.write(str(self.sensorData[0,0])+ "," + str(self.sensorData[1,0])+ ",")
                #Pos_GT_xy(4:5)
                file_gt.write(str(self.gt_pos[0,0])+ "," + str(self.gt_pos[1,0])+ ",")
                #Velocity_GT_xyz(6:8)
                file_gt.write(str(self.gt_vel[0,0])+ ',' + str(self.gt_vel[1,0])+ ',' + str(self.gt_vel[2,0])+ ',' )
                # Orientation_GT_wxyz(9:12)
                file_gt.write(str(self.gt_quat[0,0])+ ',' + str(self.gt_quat[1,0])+ ',' + str(self.gt_quat[2,0]) + ',' + str(self.gt_quat[3,0]) + ',' )
                # Time(13)
                file_gt.write(str(self.time)+',')
                #KF_quat_wxyz(14:17)
                file_gt.write(str(self.x_h[6,0])+ "," +str(self.x_h[7,0])+ "," +str(self.x_h[8,0])+ "," +str(self.x_h[9,0]))
                file_gt.write("\n")
                file_gt.close()

    def resetP(self,angle_error):


        #print('ang_error= ')
        #print(angle_error)
        # Create the reset terms for the position and velocity
        O6x9 = np.zeros([6,9])
        g_PV = np.eye(6)
        g_PV = np.hstack((g_PV,O6x9))

        qtemp= Quaternion(w_or_q=self.x_h[6],x=self.x_h[7],y=self.x_h[8],z=self.x_h[9])

        # Create the reset terms for the small angle change.NOTE: This is for Global anglular error not Local
        O3x6 = np.zeros([3,6])
        g_angle = np.eye(3) + self.makeSkew(0.5*angle_error) #@ qtemp.get_rot()
        #print('g_angle=   ')
        #print(g_angle)
        g_angle = np.hstack((O3x6, g_angle ,O3x6))

        #Create the reset terms for the sensor bias term(only angular speed and acceleration bias is used)(6x15)
        g_bias = np.eye(6)
        g_bias = np.hstack((O6x9,g_bias))

        #Create the reset matrix (15x15)
        G = np.concatenate((g_PV, g_angle, g_bias),axis=0)

        # Reset the P(Cov matrix)
        self.P = G @ self.P @ G.T

    def groundTruth(self,data):

        if self.logGT == True:
            if self.optiTrack == False or self.optiTrack_real == False:
                if self.optiTrack == False:
                    tmp_c = "iris"
                else:
                    tmp_c = "iris_optitrack"

                for x in range(0,len(data.name)):
                    if (data.name[x] == tmp_c):
                        # Positon
                        if self.gt_start == True:
                            self.gt_pos_0[0,0] = data.pose[x].position.x
                            self.gt_pos_0[1,0] = data.pose[x].position.y
                            self.gt_pos[0,0] = 0
                            self.gt_pos[0,0] = 0
                            self.gt_start = False
                        else:
                            self.gt_pos[0,0] = data.pose[x].position.x - self.gt_pos_0[0,0]
                            self.gt_pos[1,0] = data.pose[x].position.y - self.gt_pos_0[1,0]
                        
                        # Velocity
                        self.gt_vel[0] = data.twist[x].linear.x
                        self.gt_vel[1] = data.twist[x].linear.y
                        self.gt_vel[2] = data.twist[x].linear.z
                        
                        # Orientation(Quaternion)
                        self.gt_quat[0] = data.pose[x].orientation.w
                        self.gt_quat[1] = data.pose[x].orientation.x
                        self.gt_quat[2] = data.pose[x].orientation.y
                        self.gt_quat[3] = data.pose[x].orientation.z

            elif self.optiTrack == True and self.optiTrack_real == True:
                # Positon
                if self.gt_start == True:
                    self.gt_pos_0[0,0] =  data.pose.position.y
                    self.gt_pos_0[1,0] =  data.pose.position.x
                    self.gt_pos[0,0] = 0
                    self.gt_pos[0,0] = 0
                    self.gt_start = False
                else:
                    self.gt_pos[0,0] = data.pose.position.y - self.gt_pos_0[0,0]
                    self.gt_pos[1,0] = data.pose.position.x - self.gt_pos_0[1,0]
                
                # Velocity - The OptiTrack doesn't send the velocity of the object 
                self.gt_vel[0] = 0 # data.twist.linear.x
                self.gt_vel[1] = 0 # data.twist.linear.y
                self.gt_vel[2] = 0 # data.twist.linear.z
                
                # Orientation(Quaternion)
                self.gt_quat[0] = data.pose.orientation.w
                self.gt_quat[1] = data.pose.orientation.x
                self.gt_quat[2] = data.pose.orientation.y
                self.gt_quat[3] = data.pose.orientation.z
            

###################
# Spoof detection #
###################


#https://ieeexplore-ieee-org.proxy1-bib.sdu.dk/document/8654615

    # Measurements method From https://ieeexplore-ieee-org.proxy1-bib.sdu.dk/document/8654615
    def chi_cal_measurement_method(self):
        
        self.z_avg[self.z_i, :] = self.sensorData
        self.z_i += 1
        if self.z_i == self.window_size:  # Holding the window size for measurement
            self.z_i = 0

        # Part of the measurements average spoofing detection 
        self.x_avg[self.x_i,:] = self.x_h[0:3]  # only position in this situation
        self.x_i +=1
        if self.x_i == self.window_size:
            self.x_i = 0


        '''
        print('=======z_avg')
        print(np.sum(self.z_avg,axis=0)/self.window_size)
        print('=======x_avg')
        print((np.sum(self.x_avg,axis=0)/self.window_size))
        '''

        #x_h_dimMatch = np.vstack((self.x_h,np.zeros([5,1]))) # This is done like that so the dimensions fits together self.x_h.shape=(10,1) normally

        gamma = np.sum(self.z_avg,axis=0)/self.window_size - ((np.sum(self.x_avg,axis=0)/self.window_size) ) # eq. (38)
        #gamma = self.sensorData - (self.H @ x_h_dimMatch) # eq. (11)

        P_gamma = self.H @ self.P @ self.H.T + (self.R/self.window_size) # eq. (13)   TODO: re-write this  so it takes the average for the state, into account
        q = gamma.T @ np.linalg.pinv(P_gamma) @ gamma                  # eq. (14)
        

        return q

    # Innovation method From https://ieeexplore-ieee-org.proxy1-bib.sdu.dk/document/8654615 
    def chi_cal_innovation_method(self):
        
        # This is done like that so the dimensions fits together self.x_h.shape=(10,1) normally
        x_h_dimMatch = np.vstack((self.x_h,np.zeros([5,1])))
        gamma = self.sensorData - (self.H @ x_h_dimMatch) # eq. (11)

        self.gamma_array[self.P_i,:] =  gamma
        P_gamma = self.H @ self.P @ self.H.T + (self.R/self.window_size)# eq. (13) + eq. (28)

        self.P_inv_gamma_array[self.P_i,:] = np.linalg.pinv(P_gamma)

        P_inv_avg = np.sum(self.P_inv_gamma_array , axis=0) # eq.41

        gamma_avg = np.linalg.pinv(P_inv_avg) @ np.sum(self.P_inv_gamma_array @ self.gamma_array,axis=0) # eq.42

        q_inn_avg = gamma_avg.T @ P_inv_avg @ gamma_avg # eq. 43

        self.P_i += 1
        if self.P_i == self.window_size:  # Holding the window size for cov-matrix
            self.P_i = 0 
        
        return q_inn_avg



if __name__ == '__main__':
    try:
        ES_EKF()
    except rospy.ROSInterruptException as e:
        print (e)


