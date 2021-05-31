#!/usr/bin/env python3
# -*- coding: utf-8 -*-



###############################################
# Standard Imports                            #
###############################################
import time
from numpy import genfromtxt
import numpy as np
###############################################
# ROS Imports                                 #
###############################################
import rospy
import rospkg

###############################################
# MAVROS Imports                              #
###############################################
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import mavros.command

###############################################
# ROS Topic messages                          #
###############################################
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped , Point
from gazebo_msgs.msg import ModelStates



class Spoofing_OptiTrack:

    def __init__(self):

        rospy.init_node('PathTrigger_TimedSpoofing')

        self.FirstRun_dt = True
        self.t_prev = 0.0

        self.dronePose = np.zeros([2,1])

        self.time = 0.0
        self.startSpoofing_dist = 1 #[m]

        self.spoof_inc = Point()
        
        self.usingOptiTrack = False
        self.stableTest_SITL_OptiTrack = True
        self.loitering = True

        if self.usingOptiTrack == True:
            self.update_rate = 30 # [Hz]    
        else:
            self.update_rate = 10 # [Hz] 
        

        self.pathName = '../../Jes_OptiTrack/'+'path_optiTrack_AtoB'+ '.csv' # path_optiTrack_AtoB path_optiTrack_4point
        self.x_plan = genfromtxt( self.pathName, delimiter=',', usecols=(0))
        self.y_plan = genfromtxt( self.pathName, delimiter=',', usecols=(1))

        # Z ins't included because we are not interested in the height because we can't controll the height of the drone when it's flying with GPS
        self.spoof_speed_x = 0.0 # [m/s]
        self.spoof_speed_y = -1.5 # [m/s]
        
        
        

        self.trigger = Bool()
        self.trigger.data = False


        #######
        # ROS #
        #######

        #Publisher
        self.pub_trig_fly_path = rospy.Publisher('/optiTrack_runPath',Bool,queue_size=1)
        self.pub_spoof_inc =  rospy.Publisher('/offset_inc_OptiTrack',Point, queue_size=1)
        

        #Subscribe - Just to be able to get time and triggering the path the drone shall fly
        if self.usingOptiTrack == True:
            rospy.Subscriber('/vrpn_client_node/drone/pose',PoseStamped, self.update_dt)
        else:
            rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped,self.update_dt)
            rospy.Subscriber('/gazebo/model_states', ModelStates,self.get_drone_pose)

        rospy.spin()
        

    def get_drone_pose(self,data):

        tmp_c = "iris_optitrack"
        for x in range(0, len(data.name)):
            if(data.name[x] == tmp_c):
                self.dronePose[0] = data.pose[x].position.x
                self.dronePose[1] = data.pose[x].position.y


    def update_dt(self,data):
        t_secs_new = data.header.stamp.secs
        t_nsecs_new = data.header.stamp.nsecs*1.0e-9            # Convert from nsecs to secs

        t_current = t_secs_new + t_nsecs_new
        
        if self.FirstRun_dt is not True:
            self.time += t_current - self.t_prev 

        self.t_prev = t_current

        self.FirstRun_dt = False

        if(self.usingOptiTrack==True):
            self.dronePose[0] = data.pose[x].position.x
            self.dronePose[1] = data.pose[x].position.y

        self.pub_trig_fly_path.publish(self.trigger)
        self.trigger_spoofing()

    def trigger_spoofing(self):
        #if self.time >= self.spoof_time and

        print('drone pos x= ', self.dronePose[0])
        print('drone pos y= ', self.dronePose[1])

        if self.stableTest_SITL_OptiTrack == True:
            if self.dronePose[0] >= self.startSpoofing_dist and self.dronePose[0] >= -200 and self.dronePose[0] <= 200  :
                print('Spoofing - SITL !!')
                self.spoof_inc.x = self.spoof_speed_x /self.update_rate
                self.spoof_inc.y = self.spoof_speed_y /self.update_rate
                self.pub_spoof_inc.publish(self.spoof_inc)
            else:
                print('Not spoofing')
                self.spoof_inc.x = 0
                self.spoof_inc.y = 0
                self.pub_spoof_inc.publish(self.spoof_inc)
        else:
            if self.dronePose[0] >= self.startSpoofing_dist and self.usingOptiTrack == False and self.dronePose[0] <= self.x_plan:
                print('Spoofing - SITL !!')
                self.spoof_inc.x = self.spoof_speed_x /self.update_rate
                self.spoof_inc.y = self.spoof_speed_y /self.update_rate
                self.pub_spoof_inc.publish(self.spoof_inc)
            elif self.usingOptiTrack == True and self.dronePose[0] >= -2.0 and (self.dronePose[1] <= 3.1 and self.dronePose[1] >= -1.3) :
                print('Spoofing!!')
                self.spoof_inc.x = self.spoof_speed_x /self.update_rate
                self.spoof_inc.y = self.spoof_speed_y /self.update_rate
                self.pub_spoof_inc.publish(self.spoof_inc)
            else:
                print('Not spoofing')
                self.spoof_inc.x = 0
                self.spoof_inc.y = 0
                self.pub_spoof_inc.publish(self.spoof_inc)
            

if __name__ == '__main__':
    sp = Spoofing_OptiTrack()