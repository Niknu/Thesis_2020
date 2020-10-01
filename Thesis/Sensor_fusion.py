#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise







if __name__ == '__main__':


    klmFilt = KalmanFilter(dim_x=9,dim_z=9)

    dt=1.0

    I = np.eye(3,dtype=float)
    ZERO = np.zeros((3,3))
    
    pv = I*dt
    pa = I*0.5*dt**2

    P = np.hstack((I,pv,pa))
    V = np.hstack((ZERO,I,pv))
    a = np.hstack((ZERO,ZERO,I))
    

    C_gps = np.hstack((I,ZERO,ZERO))
    C_vel = np.hstack((ZERO,ZERO,ZERO))
    C_acc = np.hstack((ZERO,ZERO,I))


    klmFilt.F = np.vstack((P,V,a))
    klmFilt.H = np.vstack((C_gps,C_vel,C_acc))
    #klmFilt.Q = np.eye(klmFilt.F.shape[0],dtype=float)
    klmFilt.Q = Q_discrete_white_noise(dim=3, dt=dt ,block_size=3,order_by_dim=False)

    klmFilt.P *= 1000.0
    klmFilt.R = klmFilt.H


