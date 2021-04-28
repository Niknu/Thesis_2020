#!/usr/bin/env python

from numpy import genfromtxt
import numpy as np
import matplotlib.pyplot as plt
from math import *

from scipy.stats import chi2

import csv


alpha = 0.0001 # NOTE: This is chosen because we to say when it's 0.01%(EXSTREM case) out of the distribution
DOF = 2 # NOTE: This is 2 because we only measure x,y directions (not z yet)
threshold = chi2.ppf(1-alpha,df=DOF) 



# 0 = chi square distribution
# 1 = KF validatoin
# 2 = Ground truth data from Gazebo
# 3 = matlab-file from isaac - quat test


#chi_square = False #'else' #"chi_square" #
#kalmanVal = True # Fale

plot_no = 1


kalman_data_Path =  'KalmanValidation_'
#chi_data_path = 'Sp_OFF_Measurement_Qdefault_AtoB_1'

chi_data_path ='Sp_OFF_Measurement_AtoB_5_with_chiMeas_changed_!avg'


#kalman_data_Path += 'GPS_Qdefault_Ground_test_0_IncludeQuat_gyro3axis_0'+'_withP_Reset_DimChange_RHy' +'_Correction'#'GPS_Qmax_Ground_test_2_IncludeQuat_gyro3axis_0_Correction' #'KalmanValidation_GPS_Qdefault_AtoB_2'

#kalman_data_Path += 'GPS_Qdefault_AtoB_11_with_quat_rot_global_Correction'
kalman_data_Path += 'GPS_Qdefault_'+'BLA_test_withOUT_z_gravity'+'_Correction'#'GPS_Qdefault_test_of_corrrection_for_the_data_drift_on_IMU_raw_Prediction'
ground_truth_data = 'Kalman_posVsGPSVsGroundTruth_'+'BLA_newCovCorrection_eq_A2B_withOUTQuatError_zeroOrder'#'Kalman_posVsGPSVsGroundTruth_AtoB_11_with_quat_rot_global'

matlab_time = 'Isaac_data/in_time'+'.csv'
matlab_gyro = 'Isaac_data/in_gyro' +'.csv'
matlab_gyro_bias = 'Isaac_data/out_gyro_bias_isaac' +'.csv'




def get_OMEGA(gyro_raw):
        
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





if plot_no == 0: # chi square data
    
    chi_meas = genfromtxt( chi_data_path + '.csv', delimiter=',', usecols=(0))
    chi_innv = genfromtxt( chi_data_path + '.csv', delimiter=',', usecols=(1))
    threshold = genfromtxt(chi_data_path +'.csv', delimiter=',', usecols=(2)) 
    time = genfromtxt(chi_data_path +'.csv', delimiter=',', usecols=(3))

    if np.max(chi_meas) > np.max(chi_innv):
        dat_max = np.max(chi_meas) + 10
    else:
        dat_max = np.max(chi_innv) + 10

    fig, ax = plt.subplots(2,1,sharex=True)

    #(likelihood_bins,bins,__)= ax[0].hist(data,weights=1/(len(data))*np.ones_like(data),rwidth=0.99)

    ax[0].plot(time,chi_meas)
    ax[1].plot(time,chi_innv)



    ax[0].set_title('Chi Measurement')
    ax[1].set_title('Chi Innovation')

    ax[0].set_ylim([0,dat_max])
    ax[1].set_ylim([0,dat_max])

    ax[1].set_xlabel('Time [s]')

    #print(bins)
    #print('')
    #print(likelihood_bins)

    fig.suptitle(chi_data_path,fontsize=16)
    plt.show()
elif plot_no == 1: # Kalman Estimation
    
    #posX = genfromtxt(kalman_data_Path +'.csv', delimiter=',')
    #print(posX.shape)
    
    posX =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(0))
    posY =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(1))
    posZ =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(2))

    velX =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(3))
    velY =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(4))
    velZ =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(5))

    accB_x =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(6))
    accB_y =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(7))
    accB_z =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(8))

    gyroB_x =       genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(9))
    gyroB_y =       genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(10))
    gyroB_z =       genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(11))

    acc_data_x =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(12))
    acc_data_y =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(13))
    acc_data_z =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(14))

    gyro_data_x =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(15))
    gyro_data_y =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(16))
    gyro_data_z =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(17))

    quat_w =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(18)) 
    quat_x =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(19))
    quat_y =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(20))
    quat_z =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(21))

    cov_pos_x =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(22))
    cov_pos_y =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(23))
    cov_pos_z =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(24))

    cov_vel_x =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(25))
    cov_vel_y =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(26))
    cov_vel_z =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(27))

    cov_angle_x =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(28)) 
    cov_angle_y =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(29)) 
    cov_angle_z =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(30)) 

    cov_acc_bias_x =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(31)) 
    cov_acc_bias_y =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(32)) 
    cov_acc_bias_z =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(33)) 

    cov_gyro_bias_x =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(34)) 
    cov_gyro_bias_y =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(35)) 
    cov_gyro_bias_z =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(36)) 

    time =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(37))
    #dt =            genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(23))

    #time = range(len(posX))
    

    gyro_bias = np.array((gyroB_x.T,gyroB_y.T,gyroB_z.T))
    gyro = np.array((gyro_data_x.T,gyro_data_y.T,gyro_data_z.T))
    
    acc_bias = np.array((accB_x.T,accB_y.T,accB_z.T))
    acc = np.array((acc_data_x.T,acc_data_y.T,acc_data_z.T))

    acc_cor = acc - acc_bias
    acc_cor = acc_cor.T

    gyro_cor = gyro - gyro_bias
    gyro_cor = gyro_cor.T
    #plt.plot(time,cor[:,0],time,cor[:,1],time,cor[:,2])

    print("mean gyro_x= ",np.mean(gyro_data_x))
    print("mean gyro_y= ",np.mean(gyro_data_y))
    print("mean gyro_z= ",np.mean(gyro_data_z))

    print("variance gyro_x= ",np.var(gyro_data_x))
    print("variance gyro_y= ",np.var(gyro_data_y))
    print("variance gyro_z= ",np.var(gyro_data_z))


    N_est = 9
    M_est = 1
    
    fig_est , axis_est= plt.subplots(N_est,M_est , sharex=True)

    axis_est[0].plot(time,posX,time,posY,time,posZ)
    axis_est[1].plot(time,velX,time,velY,time,velZ)
    axis_est[2].plot(time,acc_data_x,time,acc_data_y,time,acc_data_z,)
    axis_est[3].plot(time,accB_x,time,accB_y,time,accB_z)
    axis_est[4].plot(time,acc_data_x-accB_x,time,acc_data_y-accB_y,time,acc_data_z-accB_z)
    axis_est[5].plot(time,gyro_data_x,time,gyro_data_y,time,gyro_data_z)
    axis_est[6].plot(time,gyroB_x,time,gyroB_y,time,gyroB_z)
    axis_est[7].plot(time,gyro_data_x-gyroB_x,time,gyro_data_y-gyroB_y,time,gyro_data_z-gyroB_z)
    axis_est[8].plot(time,quat_w,time,quat_x,time,quat_y,time,quat_z)
    #axis_est[7].plot(time,dt)

    

    axis_est[0].set_title("Error in Position")
    axis_est[1].set_title("Velocity - Estimation")
    axis_est[2].set_title("Acceleration raw")
    axis_est[3].set_title("Acceleration bias - Estimate")

    axis_est[4].set_title("Acceleration correct with bias")

    axis_est[5].set_title("Gyroscope raw")
    axis_est[6].set_title("Gyroscope bias - Estimate")
    axis_est[7].set_title("Gyroscope corrected with bias")
    axis_est[8].set_title("Quaterions - Estimation ")
    #axis_est[7].set_title("Timeline of dt ")

    axis_est[8].legend(['w','x','y','z'],loc="upper left" ,ncol=2)
    
    for i in range(N_est-1):
        axis_est[i].legend(['x','y','z'] , loc='upper right')

    
    axis_est[8].set_xlabel('Time [s]')

    #axis[0].set_ylim([-10,10])
    #axis[4].set_ylim([-0.01,0.01]) # Gyro raw

    fig_est.suptitle(kalman_data_Path, fontsize=16)
    
    N_cov = 5
    M_cov = 1


    fig_cov , axis_cov= plt.subplots(N_cov,M_cov , sharex=True)

    axis_cov[0].plot(time,cov_pos_x,time,cov_pos_y,time,cov_pos_z)
    axis_cov[1].plot(time,cov_vel_x,time,cov_vel_y,time,cov_vel_z)
    axis_cov[2].plot(time,cov_angle_x,time,cov_angle_y,time,cov_angle_z,)
    axis_cov[3].plot(time,cov_acc_bias_x,time,cov_acc_bias_y,time,cov_acc_bias_z)
    axis_cov[4].plot(time,cov_gyro_bias_x,time,cov_gyro_bias_y,time,cov_gyro_bias_z)
    
    axis_cov[0].set_title("$\sigma$ pos-error")
    axis_cov[1].set_title("$\sigma$ vel-error")
    axis_cov[2].set_title("$\sigma$ angle-error")
    axis_cov[3].set_title("$\sigma$ acc bias")
    axis_cov[4].set_title("$\sigma$ gyro bias")

    for i in range(N_cov):
        axis_cov[i].legend(['x','y','z'] , loc='upper right')

    axis_cov[4].set_xlabel('Time [s]')
    fig_cov.suptitle('output form the diag(P)', fontsize=16)

    #plt.plot(time,quat_w,time,quat_x,time,quat_y,time,quat_z)
    #plt.legend(['w','x','y','z'],loc="lower left" ,ncol=2)
    '''

    gyro_bias = np.array((gyroB_x.T,gyroB_y.T,gyroB_z.T))
    gyro = np.array((gyro_data_x.T,gyro_data_y.T,gyro_data_z.T))
    
    cor = gyro# - gyro_bias
    cor = cor.T
    quat = np.array((1, 0, 0, 0),dtype=np.float64).reshape((4,-1))

    quat_plot_w = np.zeros((len(time),1))
    quat_plot_x = np.zeros((len(time),1))
    quat_plot_y = np.zeros((len(time),1))
    quat_plot_z = np.zeros((len(time),1))
    
    print(cor.shape)
    
    for k in range(0,len(time)):
        
        if k == 0:
            dt = 0.01
        else:
            dt = time[k] - time[k-1]

        

        #w_norm = np.linalg.norm(gyro[k,:])*dt
        w_norm = np.linalg.norm(cor[k,:])*dt
        I = np.eye(4)
        #OMEGA = get_OMEGA(gyro[k,:]*dt)*0.5
        OMEGA = get_OMEGA(cor[k,:]*dt)*0.5

        if(w_norm >= 1e-8):
            quat = (np.cos(w_norm/2)*I + (2/w_norm)*np.sin(w_norm/2)*OMEGA) @ quat
        
        quat_plot_w[k] = quat[0]
        quat_plot_x[k] = quat[1]
        quat_plot_y[k] = quat[2]
        quat_plot_z[k] = quat[3]


    plt.plot(time,quat_plot_w,time,quat_plot_x,time,quat_plot_y,time,quat_plot_z)
    plt.legend(['w','x','y','z'])
    plt.yticks(np.arange(-1,1,0.2))

    plt.show()

    #plt.plot(time,gyro_data_x,time,gyro_data_y,time,gyro_data_z)
    plt.plot(time,cor[:,0],time,cor[:,1],time,cor[:,2])
    '''
    plt.show()
    

elif plot_no == 2: # Ground Truth



    kfX =           genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(0))
    kfY =           genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(1))
    
    gpsX =          genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(2))
    gpsY =          genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(3))
    
    gtX =           genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(4))
    gtY =           genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(5))
    
    gt_speed_x =    genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(6))
    gt_speed_y =    genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(7))
    gt_speed_z =    genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(8))
    
    gt_quat_w =     genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(9))
    gt_quat_x =     genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(10))
    gt_quat_y =     genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(11))
    gt_quat_z =     genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(12))

    time =          genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(13))

    N=1
    M=3

    fig, ax = plt.subplots(N,M)

    ax[0].plot(time,gt_speed_x,time,gt_speed_y,time,gt_speed_z)
    ax[1].plot(time,gt_quat_w,time,gt_quat_x,time,gt_quat_y,time,gt_quat_z)
    ax[2].plot(kfX,kfY,gpsX,gpsY,gtX,gtY)


    ax[0].set_title('Velocity GT')
    ax[1].set_title('Quat GT')
    ax[2].set_title('Position')
    

    ax[0].legend(['x','y','z'])
    ax[1].legend(['w','x','y','z'])
    ax[2].legend(['KF','GPS','GT'])

    ax[2].set_xlabel('x [m]')
    ax[2].set_ylabel('y [m]')
    
    fig.suptitle(ground_truth_data, fontsize=16)

    plt.show()
    
elif plot_no ==3: # matlab file test
    time = genfromtxt(matlab_time, delimiter=',', usecols=(0))

    gyro_x = genfromtxt(matlab_gyro, delimiter=',', usecols=(0))
    gyro_y = genfromtxt(matlab_gyro, delimiter=',', usecols=(1))
    gyro_z = genfromtxt(matlab_gyro, delimiter=',', usecols=(2))

    gyro = genfromtxt(matlab_gyro, delimiter=',')

    gyro_bias_x = genfromtxt(matlab_gyro_bias, delimiter=',', usecols=(0))
    gyro_bias_y = genfromtxt(matlab_gyro_bias, delimiter=',', usecols=(1))
    gyro_bias_z = genfromtxt(matlab_gyro_bias, delimiter=',', usecols=(2))

    gyro_bias = genfromtxt(matlab_gyro_bias, delimiter=',')

    quat = np.array((1, 0, 0, 0),dtype=np.float64).reshape((4,-1))


    quat_plot_w = np.zeros((len(time),1))
    quat_plot_x = np.zeros((len(time),1))
    quat_plot_y = np.zeros((len(time),1))
    quat_plot_z = np.zeros((len(time),1))
    
    cor = gyro + gyro_bias

    print(cor.shape)
    
    for k in range(0,len(time)):
        
        if k == 0:
            dt = 0.01
        else:
            dt = time[k] - time[k-1]

        

        #w_norm = np.linalg.norm(gyro[k,:])*dt
        w_norm = np.linalg.norm(cor[k,:])*dt
        I = np.eye(4)
        #OMEGA = get_OMEGA(gyro[k,:]*dt)*0.5
        OMEGA = get_OMEGA(cor[k,:]*dt)*0.5

        if(w_norm >= 1e-8):
            quat = (np.cos(w_norm/2)*I + (2/w_norm)*np.sin(w_norm/2)*OMEGA) @ quat
        
        quat_plot_w[k] = quat[0]
        quat_plot_x[k] = quat[1]
        quat_plot_y[k] = quat[2]
        quat_plot_z[k] = quat[3]




    plt.plot(time,quat_plot_w,time,quat_plot_x,time,quat_plot_y,time,quat_plot_z)
    plt.legend(['w','x','y','z'])
    plt.yticks(np.arange(-1,1,0.2))
    plt.show()


else:
    plt.plot(data)
    plt.show()








