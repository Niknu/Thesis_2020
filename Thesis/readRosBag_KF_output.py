#!/usr/bin/env python

from matplotlib import pyplot as plt
import numpy as np
import rospy
import rosbag

from scipy import stats



counter = 0

pos = np.zeros((23380, 3))
vel = np.zeros((23380, 3))
acc = np.zeros((23380, 3))
Trueth = np.zeros((23380, 1))

#with rosbag.Bag('rosBags/KF_output1min_static_new.bag') as bag:
with rosbag.Bag('rosBags/KF_output1min_static.bag') as bag:
#with rosbag.Bag('rosBags/KF_output2min_static.bag') as bag:
#with rosbag.Bag('rosBags/KF_output5min_static.bag') as bag:
#with rosbag.Bag('rosBags/KF_output2min_ground_to_hover.bag') as bag:
    for topic, msg, t in bag.read_messages(topics=['/KF_pos_est']):
        
        pos[counter,0] = msg.data[0] # x
        pos[counter,1] = msg.data[1] # y
        pos[counter,2] = msg.data[2] # z

        vel[counter,0] = msg.data[3] # x_vel
        vel[counter,1] = msg.data[4] # y_vel
        vel[counter,2] = msg.data[5] # z_vel

        acc[counter,0] = msg.data[6] # x_acc
        acc[counter,1] = msg.data[7] # y_acc
        acc[counter,2] = msg.data[8] # z_acc

        Trueth[counter,0] = 0.0
        counter +=1

        #if counter <= 10:
        #    print(msg.data)



pos = pos[0:counter,:]
vel = vel[0:counter,:]
acc = acc[0:counter,:]
Trueth = Trueth[0:counter,:]



M = 3
N = 3

### Pos ###
plt.subplot(M,N,1)
plt.plot(pos[:,0])
plt.plot(Trueth)
plt.title('X Pos')
plt.ylabel('meter')

plt.subplot(M,N,2)
plt.plot(pos[:,1])
plt.plot(Trueth)
plt.title('Y Pos')
plt.ylabel('meter')

plt.subplot(M,N,3)
plt.plot(pos[:,2])
plt.plot(Trueth)
plt.title('Z Pos')
plt.ylabel('meter')
plt.xlabel('Iteration')


### Vel ##

plt.subplot(M,N,4)
plt.plot(vel[:,0])
plt.plot(Trueth)
plt.title('X Vel')
plt.ylabel('m/s')

plt.subplot(M,N,5)
plt.plot(vel[:,1])
plt.plot(Trueth)
plt.title('Y Vel')
plt.ylabel('m/s')

plt.subplot(M,N,6)
plt.plot(vel[:,2])
plt.plot(Trueth)
plt.title('Z Vel')
plt.ylabel('m/s')
plt.xlabel('Iteration')


### Acc ###
plt.subplot(M,N,7)
plt.plot(acc[:,0])
plt.plot(Trueth)
plt.title('X Acc')
plt.ylabel('m/s^2')

plt.subplot(M,N,8)
plt.plot(acc[:,1])
plt.plot(Trueth)
plt.title('Y Acc')
plt.ylabel('m/s^2')

plt.subplot(M,N,9)
plt.plot(acc[:,2])
plt.plot(Trueth)
plt.title('Z Acc')
plt.ylabel('m/s^2')
plt.xlabel('Iterations')

plt.show()
