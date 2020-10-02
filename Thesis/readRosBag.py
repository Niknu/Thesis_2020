#!/usr/bin/env python

from matplotlib import pyplot as plt
import numpy as np
import rospy
import rosbag

from scipy import stats

#from sensor_msgs.msg import Imu, NavSatFix



counter = 0
counter_gps = 0
counter_imu = 0
initial_time_gps = None
initial_time_imu = None



values_gps = np.zeros((23380, 4))
values_imu = np.zeros((23380, 4))

avg_imu_x=0.0
avg_imu_y=0.0
avg_imu_z=0.0

gps_cov = np.zeros((3,3))
imu_cov = np.zeros((3,3))

#with rosbag.Bag('ground_to_straightLine.bag') as bag:
#with rosbag.Bag('ground_to_hover_GPS.bag') as bag:
#with rosbag.Bag('static_ground.bag') as bag:
#with rosbag.Bag('static_ground2min.bag') as bag:
with rosbag.Bag('static_ground5min.bag') as bag:
    for topic, msg, t in bag.read_messages(topics=['/mavros/global_position/global','/mavros/imu/data']):
        


        if topic == '/mavros/global_position/global': 

            if initial_time_gps is None:
                initial_time_gps = t.to_sec() 
                #print(msg.position_covariance)
                #print(' ')
                #gps_cov = np.copy(msg.position_covariance)
                #print(gps_cov.shape)
                #print(gps_cov.reshape(3,3))


            #print(topic)
            values_gps[counter_gps,0] = msg.latitude
            values_gps[counter_gps,1] = msg.longitude
            values_gps[counter_gps,2] = msg.altitude
            values_gps[counter_gps,3] = t.to_sec() - initial_time_gps

            gps_cov = np.copy(msg.position_covariance)
            
            #print(gps_cov.reshape(3,3))
            
            counter_gps +=1

        if topic == '/mavros/imu/data':

            if initial_time_imu is None:
                initial_time_imu = t.to_sec() 
                print(msg)
                #print(' ')
                #print(msg.linear_acceleration.z)

            values_imu[counter_imu,0] = msg.linear_acceleration.x
            values_imu[counter_imu,1] = msg.linear_acceleration.y
            values_imu[counter_imu,2] = msg.linear_acceleration.z
            values_imu[counter_imu,3] = t.to_sec() - initial_time_imu

            gps_cov = np.copy(msg.linear_acceleration_covariance)
            
            #print(gps_cov.reshape(3,3))
            #print(' ')
            

            counter_imu +=1




values_imu = values_imu[0:counter_imu,:]
values_gps = values_gps[0:counter_gps,:]

avg_imu_x = np.mean(values_imu[:,0])
avg_imu_y = np.mean(values_imu[:,1])
avg_imu_z = np.mean(values_imu[:,2])



## calulate positon raw from Acceleration ##



pos_x = np.zeros(len(values_imu)-1)
pos_y = np.zeros(len(values_imu)-1)
pos_z = np.zeros(len(values_imu)-1)

pos_x_prev = 0.0
pos_y_prev = 0.0
pos_z_prev = 0.0


vel_x = np.zeros(len(values_imu)-1)
vel_y = np.zeros(len(values_imu)-1)
vel_z = np.zeros(len(values_imu)-1)

vel_x_prev = 0.0
vel_y_prev = 0.0
vel_z_prev = 0.0

dt = 0.0 #np.zeros(len(values_imu))

dt_arr = np.zeros(len(values_imu-1))

for k in range(len(values_imu)-1):

    dt = values_imu[k+1,3]-values_imu[k,3] 
    dt_arr[k] = dt
    a_x = values_imu[k,0] - avg_imu_x
    a_y = values_imu[k,1] - avg_imu_y
    a_z = values_imu[k,2] - avg_imu_z


    vel_x[k] = vel_x_prev + dt*a_x
    vel_y[k] = vel_y_prev + dt*a_y
    vel_z[k] = vel_z_prev + dt*a_z


    pos_x[k] = pos_x_prev + dt*vel_x[k] + 0.5*a_x*dt**2
    pos_y[k] = pos_y_prev + dt*vel_y[k] + 0.5*a_y*dt**2
    pos_z[k] = pos_z_prev + dt*vel_z[k] + 0.5*a_z*dt**2


    vel_x_prev = vel_x[k]
    vel_y_prev = vel_y[k]
    vel_z_prev = vel_x[k]

    pos_x_prev = pos_x[k]
    pos_y_prev = pos_y[k]
    pos_z_prev = pos_z[k] 



M = 5
N = 4

### IMU ###
plt.subplot(M,N,4)
plt.plot(values_imu[:,3],values_imu[:,2])
plt.title('Z IMU')
plt.ylabel('Acceleration')

plt.subplot(M,N,3)
plt.plot(values_imu[:,3],values_imu[:,1])
plt.title('Y IMU')
plt.ylabel('Acceleration')

plt.subplot(M,N,2)
plt.plot(values_imu[:,3],values_imu[:,0])
plt.title('X IMU')
plt.ylabel('Acceleration')
plt.xlabel('Time')


### IMU with bias removed ##

plt.subplot(M,N,8)
plt.plot(values_imu[:,3],values_imu[:,2]-avg_imu_z)
plt.title('Z IMU')
plt.ylabel('Acceleration')

plt.subplot(M,N,7)
plt.plot(values_imu[:,3],values_imu[:,1]-avg_imu_y)
plt.title('Y IMU')
plt.ylabel('Acceleration')

plt.subplot(M,N,6)
plt.plot(values_imu[:,3],values_imu[:,0]-avg_imu_x)
plt.title('X IMU')
plt.ylabel('Acceleration')
plt.xlabel('Time')


### GPS ###
plt.subplot(M,N,12)
plt.plot(values_gps[:,3],values_gps[:,2])
plt.title('Altitude')
plt.ylabel('Pos')


plt.subplot(M,N,11)
plt.plot(values_gps[:,3],values_gps[:,1])
plt.title('Longitude')
plt.ylabel('Pos')


plt.subplot(M,N,10)
plt.plot(values_gps[:,3],values_gps[:,0])
plt.title('Latitude')
plt.ylabel('Pos')
plt.xlabel('Time')


### pos ###

plt.subplot(M,N,13)
plt.plot(pos_x,pos_y)
plt.title('Pos xy')
plt.ylabel('pos y')
plt.xlabel('pos x')

plt.subplot(M,N,14)
plt.plot(pos_x)
plt.title('X Pos')
plt.ylabel('Pos')

plt.subplot(M,N,15)
plt.plot(pos_y)
plt.title('Y direction')
plt.ylabel('Pos')

plt.subplot(M,N,16)
plt.plot(pos_z)
plt.title('Z direction')
plt.ylabel('Pos')

### vel ###

plt.subplot(M,N,17)
plt.plot(vel_x,vel_y)
plt.title('Vel xy')
plt.ylabel('Vel y')
plt.xlabel('Vel x')

plt.subplot(M,N,18)
plt.plot(vel_x)
plt.title('X Vel')
plt.ylabel('Vel')

plt.subplot(M,N,19)
plt.plot(vel_y)
plt.title('Y direction')
plt.ylabel('Vel')

plt.subplot(M,N,20)
plt.plot(vel_z)
plt.title('Z direction')
plt.ylabel('Vel')



plt.show()
