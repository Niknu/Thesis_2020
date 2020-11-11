#!/usr/bin/env python

import numpy as np
import rospy
from message_filters import *  # For subscribe to multiple topics and do one thing

from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import NavSatFix, Imu, MagneticField    #From the /mavros/global_position/global or mavros/global_position/raw/fix
from madgwickahrs import MadgwickAHRS
from quaternion import Quaternion 


class MadgwickROS():

    def __init__(self):

        self.FirstRun_dt = True
        self.t_prev = None
        self.dt=0.1

        self.qDot = Quaternion(1, 0, 0, 0)
        
        # --- Madgwick filter --- #
        self.beta = 0.334 # This is from his own paper where he point out where it's optimal
        self.madFilt=MadgwickAHRS(beta=self.beta,sampleperiod=1/self.dt)

        rospy.init_node('MadgwickFilter',anonymous=True)
        self.mag_data = Subscriber('/mavros/imu/mag',MagneticField)
        self.imu_data = Subscriber('/mavros/imu/data',Imu)
        self.madgwick_sub = ApproximateTimeSynchronizer([self.mag_data,self.imu_data],1,1)
        self.madgwick_sub.registerCallback(self.madgwickFilter_callback)

        self.pub_data_filter = rospy.Publisher('/MadwickFilt_quat',Float64MultiArray,queue_size=1)

        rospy.spin()



    def madgwickFilter_callback(self,magData,imuData):

        self.update_dt(imuData)

        acc = np.empty(3)
        gyro = np.empty(3)
        mag = np.empty(3)

        acc[0] = imuData.linear_acceleration.x
        acc[1] = imuData.linear_acceleration.y
        acc[2] = imuData.linear_acceleration.z

        gyro[0] = imuData.angular_velocity.x
        gyro[1] = imuData.angular_velocity.y
        gyro[2] = imuData.angular_velocity.z

        mag[0] = magData.magnetic_field.x
        mag[0] = magData.magnetic_field.y
        mag[0] = magData.magnetic_field.z

        __ , self.qDot = self.madFilt.update(gyroscope=gyro,accelerometer=acc,magnetometer=mag,sampleperiod=1/self.dt) # the output is the quat(not used) and qdot

        #print(self.qDot[0],self.qDot[1],self.qDot[2],self.qDot[3])
        #self.qDot = np.array((self.qDot[0],self.qDot[1],self.qDot[2],self.qDot[3]))
        #print(self.qDot)
        self.pub_data(self.qDot)



    def pub_data(self,data):
        #print(' ')
        
        layout = self.init_multdata()
        data_1 = Float64MultiArray(layout=layout,data=data)

        self.pub_data_filter.publish(data_1)
        #self.estPos_pub.publish(data)

    def init_multdata(self):
        msg = MultiArrayLayout()

        msg.data_offset = 0

        msg.dim = [MultiArrayDimension()]

        msg.dim[0].label= "Quat"
        msg.dim[0].size = 4
        msg.dim[0].stride = 4

        return msg

    def update_dt(self,data):
        t_secs_new = data.header.stamp.secs
        t_nsecs_new = data.header.stamp.nsecs*1.0e-9            # Convert from nsecs to secs

        t_current = t_secs_new + t_nsecs_new
        
        if self.FirstRun_dt is not True:
            self.dt = t_current - self.t_prev                   #Find the time difference (delta t)

        self.t_prev = t_current

        self.FirstRun_dt = False

if __name__ == '__main__':
    try:
        MadgwickROS()
    except rospy.ROSInterruptException as e:
        print (e)   