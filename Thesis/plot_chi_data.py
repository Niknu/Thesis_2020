#!/usr/bin/env python

from numpy import genfromtxt
import numpy as np
import matplotlib.pyplot as plt

from scipy.stats import chi2

import csv


alpha = 0.0001 # NOTE: This is chosen because we to say when it's 0.01%(EXSTREM case) out of the distribution
DOF = 2 # NOTE: This is 2 because we only measure x,y directions (not z yet)
threshold = chi2.ppf(1-alpha,df=DOF) 



# 0 = chi square distribution
# 1 = KF validatoin



#chi_square = False #'else' #"chi_square" #
#kalmanVal = True # Fale

plot_no = 1


kalman_data_Path =  'KalmanValidation_'
chi_data_path = 'Sp_OFF_Measurement_Qdefault_AtoB_1'

kalman_data_Path += 'GPS_Qmax_Ground_test_0_IncludeQuat_gyro3axis_0'+'_withP_Reset_DimChange_RHy' +'_Correction'#'GPS_Qmax_Ground_test_2_IncludeQuat_gyro3axis_0_Correction' #'KalmanValidation_GPS_Qdefault_AtoB_2'








if plot_no == 0:
    
    data = genfromtxt( chi_data_path + '.csv', delimiter=',', usecols=(0))

    (likelihood_bins,bins,__)= plt.hist(data,weights=1/(len(data))*np.ones_like(data),rwidth=0.99)
    print(bins)
    print('')
    print(likelihood_bins)

    plt.title(chi_data_path)
    plt.show()
elif plot_no == 1:
    
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

    time =          genfromtxt(kalman_data_Path +'.csv', delimiter=',', usecols=(22))




    #time = range(len(posX))
    

    N = 7
    M = 1
    fig , axis= plt.subplots(N,M , sharex=True)

    axis[0].plot(time,posX,time,posY,time,posZ)
    axis[1].plot(time,velX,time,velY,time,velZ)
    axis[2].plot(time,acc_data_x,time,acc_data_y,time,acc_data_z,)
    axis[3].plot(time,accB_x,time,accB_y,time,accB_z)
    axis[4].plot(time,gyro_data_x,time,gyro_data_y,time,gyro_data_z)
    axis[5].plot(time,gyroB_x,time,gyroB_y,time,gyroB_z)
    axis[6].plot(time,quat_w,time,quat_x,time,quat_y,time,quat_z)

    axis[0].set_title("Error in Position")
    axis[1].set_title("Velocity - Estimation")
    axis[2].set_title("Acceleration raw")
    axis[3].set_title("Acceleration bias - Estimate")
    axis[4].set_title("Gyroscope raw")
    axis[5].set_title("Gyroscope bias - Estimate")
    axis[6].set_title("Quaterions - Estimation ")

    axis[6].legend(['w','x','y','z'],loc="upper left" ,ncol=2)
    
    for i in range(N-1):
        axis[i].legend(['x','y','z'] , loc='upper right')

    fig.suptitle(kalman_data_Path, fontsize=16)
    plt.show()
    
else:
    plt.plot(data)
    plt.show()








