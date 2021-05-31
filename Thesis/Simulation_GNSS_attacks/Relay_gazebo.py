#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Relay_gazebo.py: 

"""

###############################################
# Standard Imports                            #
###############################################
import time
import threading
from math import *
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

###############################################
# Offboad Control class                       #
###############################################
class VRPNPoseRelay:
    def __init__(self, *args):
        rospy.init_node('Relay_server')

        self.sent = 0
        self.count = 0
        freq_gazebo = 250 # [hz] the gazebo freq for the position for the drone
        freq_OptiTrack = 2.5 #10 # [hz]
        self.convertToOptiTrackHz = int(freq_gazebo / freq_OptiTrack) # this is equal to 8 and this the amount of time the gazebo need to sent before it's will be sent to the drone 
                                                                    # ^ This gives a freq at around 31.25 Hz


        # 250 hz = 4ms
        # 30 hz  = 33.3ms

        #1 Publish to run offboard control
        self.spoofing_active = Bool()
        self.plan_pose = PoseStamped()
        self.rate = rospy.Rate(250)
        self.runPath = False
        self.dist_threshold = 0.1 # [m]

        #1.1 initialize the plan_pose
        self.plan_pose.pose.position.x = 0#-2.5
        self.plan_pose.pose.position.y = 0#-2.5
        self.plan_pose.pose.position.z = 0
        #1.2 Reading the path
        #if self.runPath == True:

        fileName = 'path_optiTrack_4point' # path_optiTrack_AtoB ---- path_optiTrack_4point

        self.pathName = '/home/nicolai/Desktop/'+'Jes_OptiTrack/'+fileName+ '.csv' # path_optiTrack_AtoB path_optiTrack_4point
    
        self.x_plan = genfromtxt( self.pathName, delimiter=',', usecols=(0))
        self.y_plan = genfromtxt( self.pathName, delimiter=',', usecols=(1))
        self.pathName = fileName + '.csv'
        self.path_index = 0
        

        if self.pathName == 'path_optiTrack_AtoB.csv':
            self.path_len = 1
        else:
            self.path_len = len(self.x_plan)
        
        print(self.path_len)# = 1# len(self.x_plan)


        #2 spoofing part
        self.offset_x = 0
        self.offset_y = 0
        self.offset_z = 0

        #3 Publishers
        self.pub_mocap = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.pub_path = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pub_spoof_active = rospy.Publisher('/spoofing_active_signal',Bool,queue_size=1)

        #4 Subscribers
        self.sub_vrpn_pose = rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_vrpn_pose)
        self.sub_set_offset_spoofing = rospy.Subscriber('/offset_OptiTrack',Point,self.set_offset_spoofing)
        self.sub_inc_offset_spoofing = rospy.Subscriber('/offset_inc_OptiTrack',Point,self.inc_offset_spoofing)
        self.sub_path_check = rospy.Subscriber('/gazebo/model_states', ModelStates, self.way_point_update)
        self.sub_run_path = rospy.Subscriber('/optiTrack_runPath',Bool,self.trigger_path)


        #1.3 Arm the drone
        self.set_arming()
        
        #send 100 setpoints before starting
        for i in range(0, 50):
            self.pub_path.publish(self.plan_pose)
            self.rate.sleep()

        self.set_offboard_mode()

        while not rospy.is_shutdown():
            self.pub_path.publish(self.plan_pose)
            self.rate.sleep()

    
    def set_offboard_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException(e):
            print( "service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def set_arming(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException(e):
            print( "service set_mode call failed: %s. Offboard Mode could not be set."%e)

    """
    Callbacks
    * set_offset_spoofing
    * inc_offset_spoofing
    * trigger_path
    * cb_vrpn_pose
    * way_point_update
    """


    def set_offset_spoofing(self,data):
        self.offset_x = data.x
        self.offset_y = data.y
        self.offset_z = data.z

        self.set_spoofing_signal(data)

    def inc_offset_spoofing(self,data):
        self.offset_x += data.x
        self.offset_y += data.y
        self.offset_z += data.z
        
        self.set_spoofing_signal(data)

    def set_spoofing_signal(self,data):
        
        if ((data.x + data.y + data.z) != 0.0 ):
            self.spoofing_active.data = True
        else:
            self.spoofing_active.data = False

    def trigger_path(self,in_bool):
        self.runPath = in_bool.data

    def cb_vrpn_pose(self,data):

        #tmp_c = rospy.get_namespace()
        #tmp_c = tmp_c.replace('/', '')
        #print("ns: {}".format(tmp_c))

        tmp_c = "iris_optitrack"

        #print("length --> {}".format(len(data.name)))

        for x in range(0, len(data.name)):
            #print("ns: {}".format(data.name[x]))

            if(data.name[x] == tmp_c):
                self. count += 1
                if self.count == self.convertToOptiTrackHz:
                    tmp = PoseStamped()

                    tmp.header.stamp = rospy.Time.now()
                    tmp.header.frame_id = "world"
                    tmp.header.seq = self.sent

                    tmp.pose = data.pose[x]

                    # insert the spoofing data
                    tmp.pose.position.x += self.offset_x
                    tmp.pose.position.y += self.offset_y
                    tmp.pose.position.z += self.offset_z

                    self.sent = self.sent + 1

                    self.pub_mocap.publish(tmp)
                    self.pub_spoof_active.publish(self.spoofing_active)
                    self.count = 0

    def way_point_update(self,data):
        tmp_c = "iris_optitrack"

        if (self.runPath== True):
            for x in range(0, len(data.name)):
                #print("ns: {}".format(data.name[x]))

                if(data.name[x] == tmp_c):
                    x_dist = self.plan_pose.pose.position.x - data.pose[x].position.x

                    dist = sqrt((x_dist)**2  + (self.plan_pose.pose.position.y -data.pose[x].position.y)**2)
                    print('----dist= ',dist)
                    if(self.dist_threshold >= dist ):
                        
                        if self.pathName == 'path_optiTrack_AtoB.csv':
                            self.plan_pose.pose.position.x = self.x_plan
                            self.plan_pose.pose.position.y = self.y_plan
                        else:
                            self.plan_pose.pose.position.x = self.x_plan[self.path_index]
                            self.plan_pose.pose.position.y = self.y_plan[self.path_index]
                        if (self.path_index != (self.path_len - 1)):
                            self.path_index += 1




if __name__ == '__main__':
    VRPNPR = VRPNPoseRelay()
