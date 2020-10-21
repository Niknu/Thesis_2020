#!/usr/bin/env python

from matplotlib import pyplot as plt
import numpy as np
import rospy
import rosbag

from scipy import stats



static = False

counter = 0

pos = np.zeros((23380, 3))
NED_trueth = np.zeros((23380, 3))
vel = np.zeros((23380, 3))
acc = np.zeros((23380, 3))
acc_trueth = np.zeros((23380, 3))
static_trueth = np.zeros((23380, 1))


#with rosbag.Bag('rosBags/KF_output1min_static.bag') as bag:
#with rosbag.Bag('rosBags/KF_output2min_static.bag') as bag:
with rosbag.Bag('rosBags/KF_output5min_static.bag') as bag:
#with rosbag.Bag('rosBags/KF_output2min_ground_to_hover.bag') as bag: # it's to 1.7-1.8m height
    for topic, msg, t in bag.read_messages(topics=['/KF_pos_est']):
        

        #print(msg)

        pos[counter,0] = msg.data[0] # x
        pos[counter,1] = msg.data[1] # y
        pos[counter,2] = msg.data[2] # z

        vel[counter,0] = msg.data[3] # x_vel
        vel[counter,1] = msg.data[4] # y_vel
        vel[counter,2] = msg.data[5] # z_vel

        acc[counter,0] = msg.data[6] # x_acc
        acc[counter,1] = msg.data[7] # y_acc
        acc[counter,2] = msg.data[8] # z_acc

        ### Data into the KF /true data ###

        NED_trueth[counter,0] = msg.data[9] # x
        NED_trueth[counter,1] = msg.data[10] # y
        NED_trueth[counter,2] = msg.data[11] # z

        acc_trueth[counter,0] = msg.data[12] # x_acc
        acc_trueth[counter,1] = msg.data[13] # y_acc
        acc_trueth[counter,2] = msg.data[14] # z_acc


        if static == True:
            static_trueth[counter,0] = 0.0

            
        counter +=1

        #if counter <= 10:
        #    print(msg.data)



pos = pos[0:counter,:]
vel = vel[0:counter,:]
acc = acc[0:counter,:]
static_trueth = static_trueth[0:counter,:]

NED_trueth = NED_trueth[0:counter,:]
acc_trueth = acc_trueth[0:counter,:]




M = 3
N = 3

### Pos ###
plt.subplot(M,N,1)
plt.plot(pos[:,0],label='Estimation')
if static == True:
    plt.plot(static_trueth,label='Trueth_static')
else:
    plt.plot(NED_trueth[:,0],label='KF_data_in')
plt.title('X Pos')
plt.ylabel('meter')
plt.legend()


plt.subplot(M,N,2)
plt.plot(pos[:,1],label='Estimation')
if static == True:
    plt.plot(static_trueth,label='Trueth_static')
else:
    plt.plot(NED_trueth[:,1],label='KF_data_in')
plt.title('Y Pos')
plt.ylabel('meter')
plt.legend()


plt.subplot(M,N,3)
plt.plot(pos[:,2],label='Estimation')
if static == True:
    plt.plot(static_trueth,label='Trueth_static')
else:
    plt.plot(NED_trueth[:,2],label='KF_data_in')
plt.title('Z Pos')
plt.ylabel('meter')
plt.xlabel('Iteration')
plt.legend()



### Vel ##

plt.subplot(M,N,4)
plt.plot(vel[:,0],label='Estimation')
if static == True:
    plt.plot(static_trueth,label='Trueth_static')
plt.title('X Vel')
plt.ylabel('m/s')
plt.legend()


plt.subplot(M,N,5)
plt.plot(vel[:,1],label='Estimation')
if static == True:
    plt.plot(static_trueth,label='Trueth_static')
plt.title('Y Vel')
plt.ylabel('m/s')
plt.legend()


plt.subplot(M,N,6)
plt.plot(vel[:,2],label='Estimation')
if static == True:
    plt.plot(static_trueth,label='Trueth_static')
plt.title('Z Vel')
plt.ylabel('m/s')
plt.xlabel('Iteration')
plt.legend()



### Acc ###
plt.subplot(M,N,7)
plt.plot(acc[:,0],label='Estimation')
if static == True:
    plt.plot(static_trueth,label='Trueth_static')
else:
    plt.plot(acc_trueth[:,0],label='KF_data_in')
plt.title('X Acc')
plt.ylabel('m/s^2')
plt.legend()


plt.subplot(M,N,8)
plt.plot(acc[:,1],label='Estimation')
if static == True:
    plt.plot(static_trueth,label='Trueth_static')
else:
    plt.plot(acc_trueth[:,1],label='KF_data_in')
plt.title('Y Acc')
plt.ylabel('m/s^2')
plt.legend()


plt.subplot(M,N,9)
plt.plot(acc[:,2],label='Estimation')
if static == True:
    plt.plot(static_trueth,label='Trueth_static')
else:
    plt.plot(acc_trueth[:,2],label='KF_data_in')
plt.title('Z Acc')
plt.ylabel('m/s^2')
plt.xlabel('Iterations')
plt.legend()


plt.show()
