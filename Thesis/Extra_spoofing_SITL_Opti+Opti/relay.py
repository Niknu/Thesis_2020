#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
relay.py: Controlling the setpoints, used for testing the OptiTrack system

"""

###############################################
# Standard Imports                            #
###############################################
import time
import threading
from math import *

###############################################
# ROS Imports                                 #
###############################################
import rospy
import rospkg

###############################################
# ROS Topic messages                          #
###############################################
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool

###############################################
# Offboad Control class                       #
###############################################
class VRPNPoseRelay:
    def __init__(self, *args):
        rospy.init_node('relay_server')

        self.sent = 0
        self.offset_x = 0
        self.offset_y = 0
        self.offset_z = 0
        self.spoofing_active = Bool()

        # Publishers
        self.pub_mocap = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.pub_spoof_active = rospy.Publisher('/spoofing_active_signal',Bool,queue_size=1)

        # Subscribers
        self.sub_vrpn_pose = rospy.Subscriber('/vrpn_client_node/drone/pose', PoseStamped, self.cb_vrpn_pose)
        self.sub_set_offset_spoofing = rospy.Subscriber('offset_OptiTrack',Point,self.set_offset_spoofing)
        self.sub_inc_offset_spoofing = rospy.Subscriber('offset_inc_OptiTrack',Point,self.inc_offset_spoofing)

        # Spin until the node is stopped
        rospy.spin()

    """
    Callbacks
    * cb_vrpn_pose
    * set_offset_spoofing
    * inc_offset_spoofing
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
        
        if ((data.x + data.y + data.z) == 0.0 ):
            self.spoofing_active.data = False
        else:
            self.spoofing_active.data = True

    def trigger_path(self,in_bool):
        self.runPath = in_bool.data

    def cb_vrpn_pose(self,data):
        tmp = PoseStamped()

        tmp.header.stamp = rospy.Time.now()
        tmp.header.frame_id = "world"
        tmp.header.seq = self.sent

        tmp.pose = data.pose

        # insert the spoofing data
        tmp.pose.position.x += self.offset_x
        tmp.pose.position.y += self.offset_y
        tmp.pose.position.z += self.offset_z


        self.sent = self.sent + 1

        self.pub_mocap.publish(tmp)
        self.pub_spoof_active.publish(self.spoofing_active)

    def way_point_update(self,data):
        tmp_c = "iris_optitrack"

        if (self.runPath== True):

            x_dist = self.plan_pose.pose.position.x - data.pose.position.x

            dist = sqrt((x_dist)**2  + (self.plan_pose.pose.position.y -data.pose.position.y)**2)
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
