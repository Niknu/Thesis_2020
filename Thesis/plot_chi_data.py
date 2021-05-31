#!/usr/bin/env python

from numpy import genfromtxt
import numpy as np
import matplotlib.pyplot as plt
from math import *
import os
my_path = os.path.abspath(__file__)


from scipy.stats import chi2

import csv


alpha = 0.0001 # NOTE: This is chosen because we to say when it's 0.01%(EXSTREM case) out of the distribution
DOF = 2 # NOTE: This is 2 because we only measure x,y directions (not z yet)
threshold = chi2.ppf(1-alpha,df=DOF) 


#for plot_no:
# 0 = chi square distribution
# 1 = KF validatoin
# 2 = Ground truth data from Gazebo
# 3 = matlab-file from isaac - quat test


#for env_no
# 0 = Opti
# 1 = SITL_Opti
# 2 = SITL_GNSS


# ---NOTE: Remember to swap the x+y axis for the GT from the OptiTrack, the x<->y  and y should be -y to have it right -----------

OptiTrack_data= 'Thesis/OptiTrack/'
SITL_OptiTrack_data = 'Thesis/SITL_OptiTrack'
#PX4_GNSS = 'Thesis/PX4_GNSS'

plot_no = 0 # Plot
env_no = 1  # Enviorment 
spoofing = True
spoofing_signal_plot = True

if env_no == 0:
    env_name = ' OptiTrack'
    fig_end = ''
elif env_no == 1:
    env_name = ' SITL OptiTrack'
    fig_end = '_sitl_opti'
elif env_no == 2:
    env_name =' SITL GNSS'
    fig_end = '_sitl_gnss'





ground_truth_data = 'Kalman_posVsGPSVsGroundTruth_SITL_OptiTrack_straightLine_spoofing_15'
kalman_data_Path    = 'KalmanValidation_GPS_Qdefault_SITL_OptiTrack_straightLine_spoofing_15_Correction'
chi_data_path  = 'Sp_ON_SITL_OptiTrack_straightLine_spoofing_15'

spoof_drift = chi_data_path[-2]+'.'+chi_data_path[-1]


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


def quat_to_euler(wp,xp,yp,zp):
    # output is in degree
    xroll = np.arctan2(2*(wp*xp+yp*zp),(1-2*(yp**2+zp**2))) * 180/np.pi
    ypitch = np.arcsin(2*(wp*yp-zp*xp))* 180/np.pi
    zyaw = np.arctan2(2*(wp*zp+xp*yp),(1-2*(yp**2+zp**2))) * 180/np.pi

    return xroll, ypitch, zyaw



if plot_no == 0: # chi square data
    
    chi_meas = genfromtxt( chi_data_path + '.csv', delimiter=',', usecols=(0))
    chi_innv = genfromtxt( chi_data_path + '.csv', delimiter=',', usecols=(1))
    threshold = genfromtxt(chi_data_path +'.csv', delimiter=',', usecols=(2)) 
    time = genfromtxt(chi_data_path +'.csv', delimiter=',', usecols=(3))
    spoof_active = genfromtxt(chi_data_path +'.csv', delimiter=',', usecols=(4),dtype=bool)


    if np.max(chi_meas) > np.max(chi_innv):
        y_max = np.max(chi_meas) + 10
    else:
        y_max = np.max(chi_innv) + 10

    M = 2
    N = 1    

    #(likelihood_bins,bins,__)= plt.hist(chi_meas,weights=1/(len(chi_meas))*np.ones_like(chi_meas),rwidth=0.99)
    #print(bins)
    #print('')
    #print(likelihood_bins)
    


    fig_his, ax_his= plt.subplots(M,N)

    
    hist, bin_edges = np.histogram(chi_innv,weights=1/(len(chi_meas))*np.ones_like(chi_meas))

    print(hist)
    print(bin_edges)
    
    alpha = 0.0001
    df = 1
    xdf_1 = np.linspace(chi2.ppf(alpha, df), chi2.ppf(1-alpha, df), len(chi_meas))
    ax_his[0].plot(xdf_1, chi2.pdf(xdf_1, df), label="Chi2[1]", linewidth=2)
    ax_his[1].plot(xdf_1, chi2.pdf(xdf_1, df), label="Chi2[1]", linewidth=2)
    df = 2
    xdf_2 = np.linspace(chi2.ppf(alpha, df),chi2.ppf(1-alpha, df), len(chi_meas))
    ax_his[0].plot(xdf_2,chi2.pdf(xdf_2,df), label="Chi2[2]", linewidth=2)
    ax_his[1].plot(xdf_2,chi2.pdf(xdf_2,df), label="Chi2[2]", linewidth=2)
    df = 3
    xdf_3 = np.linspace(chi2.ppf(alpha, df), chi2.ppf(1-alpha, df), len(chi_meas))
    ax_his[0].plot(xdf_3, chi2.pdf(xdf_3, df), label="Chi2[4]", linewidth=2)
    ax_his[1].plot(xdf_3, chi2.pdf(xdf_3, df), label="Chi2[4]", linewidth=2)
    
    ax_his[0].hist(chi_meas,weights=1/(len(chi_meas))*np.ones_like(chi_meas))
    ax_his[1].hist(chi_innv,weights=1/(len(chi_innv))*np.ones_like(chi_innv))

    ax_his[0].set_title('Chi Measurement - Likelihood')
    ax_his[1].set_title('Chi Innovation - Likelihood')
    ax_his[0].set_ylim(0,1)
    ax_his[1].set_ylim(0,1)
    #ax_his[0].set_xlim(0,np.max(10))
    #ax_his[1].set_xlim(0,np.max(10))

    ax_his[0].legend(['DOF=1','DOF=2','DOF=3','q$_{measurement}$'])
    ax_his[1].legend(['DOF=1','DOF=2','DOF=3','q$_{innovation}$'])

    fig_time, ax = plt.subplots(M+1,N,sharex=True)



    ax[0].plot(time,chi_meas,time,threshold)
    ax[1].plot(time,chi_innv,time,threshold)
    ax[2].plot(time,spoof_active)

    ax[0].legend(['q$_{measurement}$','T$_d$'])
    ax[1].legend(['q$_{innovation}$','T$_d$'])
    ax[2].legend(['Signal$_{'+spoof_drift+'}$'])


    ax[0].set_title('Chi Measurement')
    ax[1].set_title('Chi Innovation')
    ax[2].set_title('When spoofing is happening')

    
    ax[0].set_ylim([0,40.0])
    ax[1].set_ylim([0,40.0])
    #ax[2].set_ylim([-0.01,1.1])

    ax[0].set_yticks(np.arange(0, 40+1, 5))
    ax[1].set_yticks(np.arange(0, 40+1, 5))

    ax[0].grid()
    ax[1].grid()
    ax_his[1].set_xlabel('Values of q ')
    ax[2].set_xlabel('Time [s]')

    
    fig_his.suptitle('Likelihood of q-value -'+env_name,fontsize=16)
    fig_time.suptitle('Timeline of the q-value -'+env_name,fontsize=16)

    fig_his.savefig('LikelihoodForQ'+fig_end+'.eps',format='eps')
    fig_time.savefig('TimeLineOfQ'+fig_end+spoof_drift+'.eps',format='eps')

    plt.show()
elif plot_no == 1: # Kalman Estimation
    

    pos_KF_X =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(0))
    pos_KF_Y =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(1))
    pos_KF_Z =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(2))

    pos_GPS_X =         genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(3))
    pos_GPS_Y =         genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(4))
    pos_GPS_Z =         genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(5))

    velX =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(6))
    velY =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(7))
    velZ =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(8))

    accB_x =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(9))
    accB_y =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(10))
    accB_z =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(11))

    gyroB_x =       genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(12))
    gyroB_y =       genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(13))
    gyroB_z =       genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(14))

    acc_data_x =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(15))
    acc_data_y =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(16))
    acc_data_z =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(17))

    gyro_data_x =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(18))
    gyro_data_y =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(19))
    gyro_data_z =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(20))

    quat_w =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(21)) 
    quat_x =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(22))
    quat_y =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(23))
    quat_z =        genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(24))

    cov_pos_x =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(25))
    cov_pos_y =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(26))
    cov_pos_z =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(27))

    cov_vel_x =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(28))
    cov_vel_y =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(29))
    cov_vel_z =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(30))

    cov_angle_x =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(31)) 
    cov_angle_y =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(32)) 
    cov_angle_z =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(33)) 

    cov_acc_bias_x =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(34)) 
    cov_acc_bias_y =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(35)) 
    cov_acc_bias_z =    genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(36)) 

    cov_gyro_bias_x =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(37)) 
    cov_gyro_bias_y =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(38)) 
    cov_gyro_bias_z =   genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(39)) 
    
    delta_angle_x =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(40))
    delta_angle_y =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(41))
    delta_angle_z =     genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(42))

    time =              genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(43))
    


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


    N_est = 10
    M_est = 1
    
    fig_est , axis_est= plt.subplots(N_est,M_est , sharex=True)

    axis_est[0].plot(time,pos_KF_X,time,pos_KF_Y,time,pos_KF_Z)
    axis_est[1].plot(time,velX,time,velY,time,velZ)
    axis_est[2].plot(time,acc_data_x,time,acc_data_y,time,acc_data_z,)
    axis_est[3].plot(time,accB_x,time,accB_y,time,accB_z)
    axis_est[4].plot(time,acc_data_x-accB_x,time,acc_data_y-accB_y,time,acc_data_z-accB_z)
    axis_est[5].plot(time,gyro_data_x,time,gyro_data_y,time,gyro_data_z)
    axis_est[6].plot(time,gyroB_x,time,gyroB_y,time,gyroB_z)
    axis_est[7].plot(time,gyro_data_x-gyroB_x,time,gyro_data_y-gyroB_y,time,gyro_data_z-gyroB_z)
    axis_est[8].plot(time,delta_angle_x,time,delta_angle_y,time,delta_angle_z)
    axis_est[9].plot(time,quat_w,time,quat_x,time,quat_y,time,quat_z)

    

    axis_est[0].set_title("Position - Estimation")
    axis_est[1].set_title("Velocity - Estimation")
    axis_est[2].set_title("Acceleration raw")
    axis_est[3].set_title("Acceleration bias - Estimate")

    axis_est[4].set_title("Acceleration correct with bias")

    axis_est[5].set_title("Gyroscope raw")
    axis_est[6].set_title("Gyroscope bias - Estimate")
    axis_est[7].set_title("Gyroscope corrected with bias")
    axis_est[8].set_title("$\delta$ $\ttheta$ - Estimation ")
    axis_est[9].set_title("Quaterions - Estimation ")

    axis_est[9].legend(['w','x','y','z'],loc="upper left" ,ncol=2)
    
    for i in range(N_est-1):
        axis_est[i].legend(['x','y','z'] , loc='upper right')



    axis_est[2].set_ylabel(r'[$\frac{m}{s^2}$]')
    axis_est[3].set_ylabel(r'[$\frac{m}{s^2}$]')
    axis_est[4].set_ylabel(r'[$\frac{m}{s^2}$]')
    axis_est[5].set_ylabel(r'[$\frac{rad}{s}$]')
    axis_est[6].set_ylabel(r'[$\frac{rad}{s}$]')
    axis_est[7].set_ylabel(r'[$\frac{rad}{s}$]')
    axis_est[8].set_ylabel('rad')


    axis_est[9].set_xlabel('Time [s]')


    fig_est.suptitle(kalman_data_Path, fontsize=16)
    
    #------------ Sensor input plot -----------------

    N_sen = 3
    M_sen = 1

    fig_acc , axis_acc= plt.subplots(N_sen,M_sen , sharex=True)
    fig_gyro , axis_gyro= plt.subplots(N_sen+1,M_sen , sharex=True)
    

    axis_acc[0].plot(time,acc_data_x,time,acc_data_y,time,acc_data_z,)
    axis_acc[1].plot(time,accB_x,time,accB_y,time,accB_z)
    axis_acc[2].plot(time,acc_data_x-accB_x,time,acc_data_y-accB_y,time,acc_data_z-accB_z)


    axis_gyro[0].plot(time,gyro_data_x,time,gyro_data_y,time,gyro_data_z)
    axis_gyro[1].plot(time,gyroB_x,time,gyroB_y,time,gyroB_z)
    axis_gyro[2].plot(time,gyro_data_x-gyroB_x,time,gyro_data_y-gyroB_y,time,gyro_data_z-gyroB_z)
    axis_gyro[3].plot(time,delta_angle_x,time,delta_angle_y,time,delta_angle_z)


    
    axis_acc[0].legend(['x','y','z'],loc='center left', bbox_to_anchor=(1, 0.5))
    axis_gyro[0].legend(['x','y','z'],loc='center left', bbox_to_anchor=(1, 0.5))


    axis_acc[0].set_title("Acceleration raw")
    axis_acc[1].set_title("Acceleration bias - Estimate")
    axis_acc[2].set_title("Acceleration correct with bias")

    axis_gyro[0].set_title("Gyroscope raw")
    axis_gyro[1].set_title("Gyroscope bias - Estimate")
    axis_gyro[2].set_title("Gyroscope corrected with bias")
    axis_gyro[3].set_title("$\delta$ $\ttheta$ - Estimation ")

    for x in range(0,N_sen):
        axis_acc[x].set_ylabel(r'[$\frac{m}{s^2}$]')
        axis_gyro[x].set_ylabel(r'[$\frac{rad}{s}$]')
    
    axis_gyro[3].set_ylabel('rad')

    axis_acc[2].set_xlabel('Time [s]')
    axis_gyro[3].set_xlabel('Time [s]')

    fig_acc.suptitle('Accelerometer input -'+ env_name, fontsize=16)
    fig_gyro.suptitle('Gyroscope input -'+ env_name, fontsize=16)

    # ------------ Cov plot --------------
    N_cov = 5
    M_cov = 1


    fig_cov , axis_cov= plt.subplots(N_cov,M_cov , sharex=True)

    
    cov_pos_x = np.sqrt(cov_pos_x)
    cov_pos_y = np.sqrt(cov_pos_y)
    cov_pos_z = np.sqrt(cov_pos_z)    
    cov_vel_x = np.sqrt(cov_vel_x)
    cov_vel_y = np.sqrt(cov_vel_y)
    cov_vel_z = np.sqrt(cov_vel_z)    
    cov_angle_x = np.sqrt(cov_angle_x)   
    cov_angle_y = np.sqrt(cov_angle_y)   
    cov_angle_z = np.sqrt(cov_angle_z)   
    cov_acc_bias_x = np.sqrt(cov_acc_bias_x)    
    cov_acc_bias_y = np.sqrt(cov_acc_bias_y)    
    cov_acc_bias_z = np.sqrt(cov_acc_bias_z)    
    cov_gyro_bias_x = np.sqrt(cov_gyro_bias_x)   
    cov_gyro_bias_y = np.sqrt(cov_gyro_bias_y)   
    cov_gyro_bias_z = np.sqrt(cov_gyro_bias_z)


    axis_cov[0].plot(time,3*cov_pos_x,time,3*cov_pos_y,time,3*cov_pos_z)
    axis_cov[1].plot(time,3*cov_vel_x,time,3*cov_vel_y,time,3*cov_vel_z)
    axis_cov[2].plot(time,3*cov_angle_x,time,3*cov_angle_y,time,3*cov_angle_z,)
    axis_cov[3].plot(time,3*cov_acc_bias_x,time,3*cov_acc_bias_y,time,3*cov_acc_bias_z)
    axis_cov[4].plot(time,3*cov_gyro_bias_x,time,3*cov_gyro_bias_y,time,3*cov_gyro_bias_z)
    
    axis_cov[0].set_title("3*$\sigma$ pos-error")
    axis_cov[1].set_title("3*$\sigma$ vel-error")
    axis_cov[2].set_title("3*$\sigma$ angle-error")
    axis_cov[3].set_title("3*$\sigma$ acc bias")
    axis_cov[4].set_title("3*$\sigma$ gyro bias")

    
    axis_cov[0].set_ylabel('[m]')
    axis_cov[1].set_ylabel(r'[$\frac{m}{s}$]')
    axis_cov[2].set_ylabel('Radians ')
    axis_cov[3].set_ylabel(r'[$\frac{m}{s^2}$] ')
    axis_cov[4].set_ylabel(r'[$\frac{rad}{s}$]  ')
    
    
    axis_cov[0].legend(['x','y','z'] ,loc='center left', bbox_to_anchor=(1, 0.5))

    axis_cov[4].set_xlabel('Time [s]')
    fig_cov.suptitle('The estimation of the std. dev -' + env_name, fontsize=16)


    axis_cov[0].set_yticks(np.arange(0, max(3*cov_pos_y)+1, 1.0))
    axis_cov[1].set_yticks(np.arange(0, max(3*cov_vel_y)+1, 2.0))
    axis_cov[2].set_yticks(np.arange(0, max(3*cov_angle_y)+1, 20.0))
    axis_cov[3].set_yticks(np.arange(0, max(3*cov_acc_bias_y)+1, 2.0))
    axis_cov[4].set_yticks(np.arange(0, max(3*cov_gyro_bias_y)+1, 2.0))



    for n in range(0,N_sen):
        axis_acc[n].grid()
        axis_gyro[n].grid()
    axis_gyro[3].grid()
        

    for n in range(0,N_cov):
        axis_cov[n].grid()
    
    for n in range(0,N_est):
        axis_est[n].grid()
    


    
    fig_acc.savefig('Acc'+fig_end+'.eps',format='eps')
    fig_gyro.savefig('Gyro'+fig_end+'.eps',format='eps')
    fig_cov.savefig('Cov_est'+fig_end+'.eps',format='eps')
    
    
    plt.tight_layout()
    
    plt.show()

elif plot_no == 2: # Ground Truth

    
    

    if env_no != 2:
        # OptiTrack
        waypoint_x = np.array((0,0,2,2))
        waypoint_y = np.array((0,2,2,0))
    else:
        waypoint_x = np.array((0,0,4,4))
        waypoint_y = np.array((0,4,4,0))


    # ---NOTE: Remember to swap the x+y axis for the GT from the OptiTrack, the x<->y  and y should be -y to have it right -----------
    

    kfX =           genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(0))
    kfY =           genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(1))
    
    gpsX =          genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(2))
    gpsY =          genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(3))
    
    gtX =           genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(4))
    gtY =           genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(5))
    
    gt_speed_x =    genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(6))
    gt_speed_y =    genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(7))
    gt_speed_z =    genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(8))
    
    # speed from the KF_estimation
    kf_speed_x = genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(6))
    kf_speed_y = genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(7))
    kf_speed_z = genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(8))

    gt_quat_w =     genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(9))
    gt_quat_x =     genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(10))
    gt_quat_y =     genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(11))
    gt_quat_z =     genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(12))

    time =          genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(13))

    KF_quat_w =        genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(14)) 
    KF_quat_x =        genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(15))
    KF_quat_y =        genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(16))
    KF_quat_z =        genfromtxt(ground_truth_data +'.csv', delimiter=',', usecols=(17))

    #print(len(KF_quat_w))
    #print(len(gt_quat_w))

    KF_xroll, KF_ypitch, KF_zyaw = quat_to_euler(KF_quat_w, KF_quat_x, KF_quat_y ,KF_quat_z )
    gt_xroll, gt_ypitch, gt_zyaw = quat_to_euler(gt_quat_w, gt_quat_x, gt_quat_y ,gt_quat_z )

    N=1
    M=3

    fig, ax = plt.subplots(N,M)
    

    ax[0].plot(time,gt_speed_x,time,gt_speed_y)
    ax[0].plot(time,kf_speed_x,time,kf_speed_y)
    ax[1].plot(time,gt_quat_w,time,gt_quat_x,time,gt_quat_y,time,gt_quat_z)
    ax[1].plot(time,KF_quat_w,time,KF_quat_x,time,KF_quat_y,time,KF_quat_z)



    ax[2].plot(kfX,kfY,gpsX,gpsY,gtX,gtY)
    if spoofing == False:
        ax[2].plot(waypoint_x,waypoint_y, marker='*',linestyle='None',color='k')


    

    ax[0].set_title('Velocity GT')
    ax[1].set_title('Quat GT')
    ax[2].set_title('Position')
    
    ax[0].legend(['x$_{GT}$','y$_{GT}$','x$_{ES-EKF}$','y$_{ES-EKF}$'], ncol=2)
    ax[1].legend(['w$_{GT}$','x$_{GT}$','y$_{GT}$','z$_{GT}$','w$_{ES-EKF}$','x$_{ES-EKF}$','y$_{ES-EKF}$','z$_{ES-EKF}$'],ncol=2)
    ax[2].legend(['ES-EKF','GPS','GT','Waypoints'])

    ax[2].set_xlabel('x [m]')
    ax[2].set_ylabel('y [m]')

    ax[1].set_ylim((-1.1,1.1))
    ax[0].set_xlabel('time [s]')
    ax[1].set_xlabel('time [s]')

    #print(np.max(gtY))
    #print(np.min(gtY))

    #if (np.max(gtY) <= 5 and np.max(gtY) >= 0) or (np.min(gtY) >= -5 and np.min(gtY) <= 0):
    #    ax[2].set_ylim((-5,5))
    
    #fig.suptitle(ground_truth_data, fontsize=16)

    for n in range(0,M):
        ax[n].grid()


    fig.suptitle('The 3 navigation states ES-EKF -'+ env_name, fontsize=16)

    ### ----------------------- Only the position ---------------------------------------###

    M_pos=1
    N_pos=1

    fig_pos, ax_pos = plt.subplots(N_pos,M_pos)


    ax_pos.plot(kfX,kfY,gpsX,gpsY,gtX,gtY)


    if spoofing == False:
        ax_pos.plot(waypoint_x,waypoint_y, marker='*',linestyle='None',color='k')
    elif spoofing_signal_plot == True:
        ax_pos.plot(gtX,-gtY+gpsY,marker=',',color='k')


    ax_pos.set_title('Position')
    
    if spoofing == False:
        ax_pos.legend(['ES-EKF','GPS','GT','Waypoints'])
    elif spoofing_signal_plot == True:
        ax_pos.legend(['ES-EKF','GPS','GT',r'Spoofing Signal$_{'+spoof_drift+'}$'],loc='lower left',bbox_to_anchor=(0.0, 0.)) #bbox_to_anchor=(0.75, 0.)
    else:
        ax_pos.legend(['ES-EKF','GPS','GT'])

    if np.max(gtY)<1:
        ax_pos.set_ylim([-1,1])

    ax_pos.set_xlabel('x [m]')
    ax_pos.set_ylabel('y [m]')
    
    ax_pos.grid()

    
    fig_pos.suptitle(' Path of the drone  -'+ env_name, fontsize=16)


    ####---------------------- The orientation in quat  ---------------------------------###

    N_quat=4
    M_quat=1
    fig_quat, ax_quat = plt.subplots(N_quat,M_quat,sharex=True)

    ax_quat[0].plot(time,gt_quat_w,time,KF_quat_w)
    ax_quat[1].plot(time,gt_quat_x,time,KF_quat_x)
    ax_quat[2].plot(time,gt_quat_y,time,KF_quat_y)
    ax_quat[3].plot(time,gt_quat_z,time,KF_quat_z)

    ax_quat[0].set_title('Quaternion value w')
    ax_quat[1].set_title('Quaternion value x')
    ax_quat[2].set_title('Quaternion value y')
    ax_quat[3].set_title('Quaternion value z')

    for n in range(0,N_quat):
        ax_quat[n].grid()
    
    
    ax_quat[0].legend(['GT','ES-EKF'],loc='lower left', bbox_to_anchor=(0.0, 0.5))

    ax_quat[3].set_xlabel('Time [s]')
    
    fig_quat.suptitle('The Quaternion values ES-EKF vs. GT -'+ env_name, fontsize=16)

    ####---------------------- The orientation in eluer angles  ---------------------------------###

    fig_ang, ax_ang = plt.subplots(M,N,sharex=True)
    
    ax_ang[0].plot(time,KF_xroll,time,gt_xroll)
    ax_ang[1].plot(time,KF_ypitch,time,gt_ypitch)
    ax_ang[2].plot(time,KF_zyaw,time,gt_zyaw)

    ax_ang[0].set_title('roll$_x$')
    ax_ang[1].set_title('pitch$_y$')
    ax_ang[2].set_title('yaw$_z$')

    ax_ang[0].set_ylabel('degrees$\circ$')
    ax_ang[1].set_ylabel('degrees$\circ$')
    ax_ang[2].set_ylabel('degrees$\circ$')

    ax_ang[2].set_xlabel('time [s]')


    ax_ang[0].legend(['ES-EKF','gt'],loc='lower left', bbox_to_anchor=(0.0, 0.7))



    for n in range(0,M):
        ax_ang[n].grid()

    fig_ang.suptitle('Orientation of the drone in euler angles -'+ env_name, fontsize=16)



    ####---------------------- The speed vs  ---------------------------------###

    fig_speed, ax_speed = plt.subplots(M-1,N,sharex=True)

    ax_speed[0].plot(time,gt_speed_x)
    ax_speed[1].plot(time,gt_speed_y)
    #ax_speed[2].plot(time,gt_speed_z)

    ax_speed[0].plot(time,kf_speed_x)
    ax_speed[1].plot(time,kf_speed_y)
    #ax_speed[2].plot(time,kf_speed_z)

    
    ax_speed[0].set_title('Velocity for the drone in x-axis')
    ax_speed[1].set_title('Velocity for the drone in y-axis')
    #ax_speed[2].set_title('Velocity for the drone in z-axis')
    
    ax_speed[0].set_ylabel(r'Velocity [$\frac{m}{s}$]')
    ax_speed[1].set_ylabel(r'Velocity [$\frac{m}{s}$]')
    #ax_speed[2].set_ylabel(r'$\frac{m}{s}$')

    #ax_speed[2].set_xlabel('time [s]')

    ax_speed[1].set_xlabel('time [s]')

    ax_speed[0].legend(['GT','ES-EKF'],loc='lower right', bbox_to_anchor=(1.0, 0.7))
    


    for n in range(0,M-1):
        ax_speed[n].grid()

    fig_speed.suptitle('Speed of the drone -' + env_name, fontsize=16)

    ####----------------------  RMSE+MEA calculation  ---------------------------------###

    # Pos
    n = len(kfX)
    RMSE_pos_KFvsGPS_x =  np.sqrt((1/n)*np.sum((kfX - gpsX)**2))
    RMSE_pos_KFvsGPS_y =  np.sqrt((1/n)*np.sum((kfY - gpsY)**2))

    RMSE_pos_KFvsGT_x =  np.sqrt((1/n)*np.sum((kfX - gtX)**2))
    RMSE_pos_KFvsGT_y =  np.sqrt((1/n)*np.sum((kfY - gtY)**2))
    
    #speed
    RMSE_speed_KFvsGT_x = np.sqrt((1/n)*np.sum((kf_speed_x - gt_speed_x)**2))
    RMSE_speed_KFvsGT_y = np.sqrt((1/n)*np.sum((kf_speed_y - gt_speed_y)**2))
    #RMSE_speed_KFvsGT_z = np.sqrt((1/n)*np.sum((kf_speed_z - gt_speed_z)**2))


    #quat
    RMSE_quat_KFvsGT_w = np.sqrt((1/n)*np.sum((KF_quat_w - gt_quat_w)**2))
    RMSE_quat_KFvsGT_x = np.sqrt((1/n)*np.sum((KF_quat_x - gt_quat_x)**2))
    RMSE_quat_KFvsGT_y = np.sqrt((1/n)*np.sum((KF_quat_y - gt_quat_y)**2))
    RMSE_quat_KFvsGT_z = np.sqrt((1/n)*np.sum((KF_quat_z - gt_quat_z)**2))

    #euler
    RMSE_euler_KFvsGT_x = np.sqrt((1/n)*np.sum((KF_xroll - gt_xroll)**2))
    RMSE_euler_KFvsGT_y = np.sqrt((1/n)*np.sum((KF_ypitch - gt_ypitch)**2))
    RMSE_euler_KFvsGT_z = np.sqrt((1/n)*np.sum((KF_zyaw - gt_zyaw)**2))

    # ------------------------ MEA ---------------------------------------
    # Pos
    n = len(kfX)
    MEA_pos_KFvsGPS_x =  (1/n)*np.sum(np.abs(kfX - gpsX))
    MEA_pos_KFvsGPS_y =  (1/n)*np.sum(np.abs(kfY - gpsY))

    MEA_pos_KFvsGT_x =  (1/n)*np.sum(np.abs(kfX - gtX))
    MEA_pos_KFvsGT_y =  (1/n)*np.sum(np.abs(kfY - gtY))
    #speed
    MEA_speed_KFvsGT_x = (1/n)*np.sum(np.abs(kf_speed_x - gt_speed_x))
    MEA_speed_KFvsGT_y = (1/n)*np.sum(np.abs(kf_speed_y - gt_speed_y))
    #quat
    MEA_quat_KFvsGT_w = (1/n)*np.sum(np.abs(KF_quat_w - gt_quat_w))
    MEA_quat_KFvsGT_x = (1/n)*np.sum(np.abs(KF_quat_x - gt_quat_x))
    MEA_quat_KFvsGT_y = (1/n)*np.sum(np.abs(KF_quat_y - gt_quat_y))
    MEA_quat_KFvsGT_z = (1/n)*np.sum(np.abs(KF_quat_z - gt_quat_z))
    #euler
    MEA_euler_KFvsGT_x = (1/n)*np.sum(np.abs(KF_xroll - gt_xroll))
    MEA_euler_KFvsGT_y = (1/n)*np.sum(np.abs(KF_ypitch - gt_ypitch))
    MEA_euler_KFvsGT_z = (1/n)*np.sum(np.abs(KF_zyaw - gt_zyaw))
    

    print('pos_x: KF vs GPS  RMSE= ',RMSE_pos_KFvsGPS_x)
    print('pos_y: KF vs GPS  RMSE= ',RMSE_pos_KFvsGPS_y)
    print('pos_x: KF vs GT   RMSE= ',RMSE_pos_KFvsGT_x)
    print('pos_y: KF vs GT   RMSE= ',RMSE_pos_KFvsGT_y)
    print('')
    print('speed_x: KF vs GT RMSE= ',RMSE_speed_KFvsGT_x)
    print('speed_y: KF vs GT RMSE= ',RMSE_speed_KFvsGT_y)
    #print('speed_z: KF vs GT RMSE= ',RMSE_speed_KFvsGT_z)
    print('')
    print('quat_w RMSE= ',RMSE_quat_KFvsGT_w)
    print('quat_x RMSE= ',RMSE_quat_KFvsGT_x)
    print('quat_y RMSE= ',RMSE_quat_KFvsGT_y)
    print('quat_z RMSE= ',RMSE_quat_KFvsGT_z)
    print('')
    print('euler_x RMSE= ',RMSE_euler_KFvsGT_x)
    print('euler_y RMSE= ',RMSE_euler_KFvsGT_y)
    print('euler_z RMSE= ',RMSE_euler_KFvsGT_z)
    print('')
    print('--- MEA ---')
    print('')
    print('pos_x: KF vs GPS  MEA= ',MEA_pos_KFvsGPS_x)
    print('pos_y: KF vs GPS  MEA= ',MEA_pos_KFvsGPS_y)
    print('pos_x: KF vs GT   MEA= ',MEA_pos_KFvsGT_x)
    print('pos_y: KF vs GT   MEA= ',MEA_pos_KFvsGT_y)
    print('')
    print('speed_x: KF vs GT MEA= ',MEA_speed_KFvsGT_x)
    print('speed_y: KF vs GT MEA= ',MEA_speed_KFvsGT_y)
    #print('speed_z: KF vs GT MEA= ',MEA_speed_KFvsGT_z)
    print('')
    print('quat_w MEA= ',MEA_quat_KFvsGT_w)
    print('quat_x MEA= ',MEA_quat_KFvsGT_x)
    print('quat_y MEA= ',MEA_quat_KFvsGT_y)
    print('quat_z MEA= ',MEA_quat_KFvsGT_z)
    print('')
    print('euler_x MEA= ',MEA_euler_KFvsGT_x)
    print('euler_y MEA= ',MEA_euler_KFvsGT_y)
    print('euler_z MEA= ',MEA_euler_KFvsGT_z)






    # ---------- Saving files into eps format
    fig_pos.savefig('path'+fig_end+'_'+ground_truth_data[-2:]+'.eps',format='eps')
    fig_quat.savefig('quat'+fig_end+'.eps',format='eps')
    fig_ang.savefig('orientation'+fig_end+'.eps',format='eps')
    fig_speed.savefig('speed'+fig_end+'.eps',format='eps')

    plt.show()

elif plot_no ==3: # matlab file test Isaac
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








