# -*- coding: utf-8 -*-
#!/usr/bin/env python

import signal
import sys

from ecef2geodtic import *  # 1st line is to run this file
from utm import utmconv  # <---- This is from Kjeld Intro 2 Drone Tech

import math
import rospy
import message_filters
from sensor_msgs.msg import NavSatFix       #From the /mavros/global_position/global or mavros/global_position/raw/fix
from geographic_msgs.msg import GeoPoint    #Part of the HIL_GPS
import std_msgs.msg
from mavros_msgs.msg import HilGPS
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
#load all messages and service from MAVROS
from mavros_msgs.msg import *
from mavros_msgs.srv import *

class SpoofingClass():

    def __init__(self):

        self.count = 0

        self.GEO_class = ecefGeo_class()
        self.uc = utmconv() # <---- from Kjelds lib
        self.t_prev = 0.0
        self.t_current = 0.0
        self.dt = 0.0
        self.data = None
        self.FirstRun_dt = True
        self.Pub_data = False   # <------ Remove ?
        self.FirstRun_pos = True
        self.FirstRun_pub = True
        self.gps_vel = 0.0 # [m/s]
        self.cog_gps = 0.0 #[cdeg]
        self.cog_amount_steps = 5.0 # [cdeg]
        self.cog_step_count = 0
        self.cog_step_dir = (1.0871889239775937e-05)/10000.0 #/10.0#1.0
        self.cog_current = None
        self.cog_drone_start = None
        self.cog_drone_end = None

        self.h = std_msgs.msg.Header()

        self.speed_x = 0.0 #[m/s]
        self.speed_y = 0.0 #[m/s]

        #The data from the raw gps signal
        self.gps_in_lat = 0.0
        self.gps_in_lon = 0.0
        self.gps_in_alt = 0.0

        # The xyz for ecef
        self.ecef_x = 0.0
        self.ecef_y = 0.0
        self.ecef_z = 0.0

        #The output from utm
        self.e1 = 0.0
        self.n1 = 0.0
        self.hemisphere = None
        self.zone = None


        #The spoofing data that will be sent to the HIL_GPS
        self.gps_out_lat = 0.0
        self.gps_out_lon = 0.0
        self.gps_out_alt = 0.0


        #init the node and subscribe to the raw signal from the gps
        rospy.init_node('get_GPS_data',anonymous=True)


        #### -------- Check if mavros/global_position/raw/fix gives better results ----- ########
        xy_pos = message_filters.Subscriber('/mavros/global_position/global',NavSatFix) #The '/mavros/global_position/global' used also the data from the IMU (http://wildfirewatch.elo.utfsm.cl/ros-topology/#global_gps)
        z_pos = message_filters.Subscriber('/mavros/altitude',Altitude)
        sub_topics= message_filters.ApproximateTimeSynchronizer([xy_pos,z_pos],10.0,0.1,allow_headerless=True)
        sub_topics.registerCallback(self.load_data)

        #Create publisher to the HIL gps to send spoofing signal
        self.pub_spoofing_signal = rospy.Publisher('/mavros/hil/gps',HilGPS,queue_size=10) # remember to 'param set MAV_USEHILGPS 1' in the px4 before this can work
        #self.pub_spoofing_signal = rospy.Publisher('/SP_TEST123213',HilGPS,queue_size=10) # tester

        self.change_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)

        # The amount of time it should publish data
        self.rosRate= rospy.Rate(1) # 5 Hz

        #rospy.spin()

        while not rospy.is_shutdown():
            self.publish_data()
            self.rosRate.sleep()

    def get_pos(self):
        self.gps_in_lat = self.data.latitude
        self.gps_in_lon = self.data.longitude
        self.gps_in_alt = self.data.altitude

    def setUSEHILGPS(self,setdata):
        rospy.wait_for_service('/mavros/param/set')
        try:
            myparam = ParamValue()
            myparam.integer = setdata
            out = self.change_param("MAV_USEHILGPS", myparam)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed")

    def setSIM_BLOCK_GPS(self,setdata):
        rospy.wait_for_service('/mavros/param/set')
        try:
            myparam = ParamValue()
            myparam.integer = setdata
            out = self.change_param("SIM_GPS_BLOCK", myparam)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed")

    def load_data(self,xy_data,z_data):

        self.get_dt(xy_data)

        self.transform_gps_xyz_ECEF(xy_data,z_data)
        #self.transform_gps_xyz_UTM(xy_data,z_data)

        # Remember to set the Cog data in publish_data
        #self.publish_data()

        #print ('----','Pos= ',self.gps_out_lat, self.gps_out_lon, self.gps_out_alt,'speed= ',self.gps_vel)
        #print ('----','Pos= ',self.gps_out_lat, self.gps_out_lon, self.gps_out_alt,'speed= ',self.gps_vel,'COG= ',self.cog_drone_current)
        #print''

    def get_dt(self,data):
        t_secs_new = data.header.stamp.secs
        t_nsecs_new = data.header.stamp.nsecs*1.0e-9            # Convert from nsecs to secs

        self.t_current = t_secs_new + t_nsecs_new

        if self.FirstRun_dt is not True:
            self.dt = self.t_current - self.t_prev              #Find the time difference (delta t)

        self.t_prev = self.t_current

        self.FirstRun_dt = False

    def transform_gps_xyz_ECEF(self,xy_data,z_data):

        #This is done so we have the
        if self.FirstRun_pos is True:
            self.gps_in_lat = xy_data.latitude
            self.gps_in_lon = xy_data.longitude
            self.gps_in_alt = z_data.amsl # Above Mean Sea Level
            self.FirstRun_pos = False
            #Transform Geo 2 ECEF
            self.ecef_x,self.ecef_y,self.ecef_z = self.GEO_class.GeoToEcef(self.gps_in_lat,self.gps_in_lon,self.gps_in_alt)
        else:
            self.ecef_x,self.ecef_y,self.ecef_z = self.GEO_class.GeoToEcef(self.gps_out_lat,self.gps_out_lon,self.gps_in_alt)

        self.speed_x = 0.5  #[m/s]
        self.speed_y = 0.0  #[m/s]

        #Add offset
        x , y = self.GEO_class.AddOffset_xy(self.ecef_x,self.ecef_y,self.dt,self.speed_x,self.speed_y)
        self.ecef_x = x
        self.ecef_y = y

        #Transform ECEF 2 Geo
        lat, lon, alt = self.GEO_class.Ecef2Geo(self.ecef_x,self.ecef_y,self.ecef_z)
        self.gps_out_lat = lat
        self.gps_out_lon = lon
        self.gps_out_alt = self.gps_in_alt #alt

        #Set the gps msg
        self.gps_vel = math.sqrt(self.speed_x**2.0+self.speed_y**2.0)# [m/s]

    def transform_gps_xyz_UTM(self,xy_data,z_data):

        # For the first run time
        if self.FirstRun_pos is True:
            self.gps_in_lat = xy_data.latitude
            self.gps_in_lon = xy_data.longitude
            self.gps_in_alt = z_data.amsl # Above Mean Sea Level
            self.FirstRun_pos = False

            # convert from geodetic to UTM
            (self.hemisphere, self.zone, letter, self.e1, self.n1) = self.uc.geodetic_to_utm(self.gps_in_lat,self.gps_in_lon)
            #print('first Lat =',self.gps_in_lat,'first lon = ',self.gps_in_lon)
            NorthRef = 90.0
            self.cog_drone_start = math.degrees(math.atan2(self.n1,self.e1)) # NorthRef - math.degrees(math.atan2(self.n1,self.e1))

            #print('############cog_raw= ',self.cog_drone_start)


            if(self.cog_drone_start < 0.0):
                self.cog_drone_start = self.cog_drone_start + 360.0

            #Checking if the offset + cog_drone_start > 360.0
            if(self.cog_drone_start+(self.cog_amount_steps*self.cog_step_dir)>359.99):
                self.cog_drone_end = self.cog_drone_start+(self.cog_amount_steps*self.cog_step_dir) - 360.0
            else:
                self.cog_drone_end = self.cog_drone_start+(self.cog_amount_steps*self.cog_step_dir)

            self.cog_drone_current = self.cog_drone_start

            #print('COG_start= ',self.cog_drone_start,'COG_end= ',self.cog_drone_end)
        else:
            # After first run time
            (self.hemisphere, self.zone, letter, self.e1, self.n1) = self.uc.geodetic_to_utm (self.gps_out_lat,self.gps_out_lon)



        #self.speed_x = 0.5  #[m/s]
        #self.speed_y = 0.5 #[m/s]


        # Set the an anglular offset with polar coordinates and convert it back to cartesian coordinates
        dt = self.dt
        self.gps_vel = 0.1 # [m/S]
        #off_x = self.speed_x*dt
        #off_y = self.speed_y*dt
        off_set = self.gps_vel * dt
        r = (math.sqrt((self.e1)**2+(self.n1)**2))+off_set
        e_bef = self.e1
        n_bef = self.n1

        #print('east before= ',self.e1,'north before= ',self.n1, 'Hemisphere= ',self.hemisphere, 'Zone= ',self.zone, 'letter= ',letter)

        #Check if max angle offset  is reached
        if(self.cog_step_count!=self.cog_amount_steps):
            self.cog_drone_current = self.cog_drone_current + self.cog_step_dir
            self.cog_step_count += 1
            print 'increase by 1'
            # Secure that the angle doesn't go above 360deg
            if(self.cog_drone_current > 359.99):
                self.cog_drone_current = self.cog_drone_current - 360.0

        self.e1 = r*math.cos(math.radians(self.cog_drone_current)) # 'x-axis'
        self.n1 = r*math.sin(math.radians(self.cog_drone_current)) # 'y-axis'


        #print('east after= ',self.e1,'north after= ',self.n1,'r= ',r,'angle= ',self.cog_drone_current)
        #print('east Diff= ',self.e1-e_bef,'north Diff= ',self.n1-n_bef)



        (lat, lon) = self.uc.utm_to_geodetic (self.hemisphere, self.zone, self.e1, self.n1)
        self.gps_out_lat = round(lat,7)
        self.gps_out_lon = round(lon,7)
        self.gps_out_alt = round(self.gps_in_alt,7) #alt

    def publish_data(self):

        mToCm = 10.0
        if self.FirstRun_pub is True:
            self.setUSEHILGPS(1)
            self.setSIM_BLOCK_GPS(1)
            self.FirstRun_pub = False

        # information about the variables https://mavlink.io/en/messages/common.html#HIL_GPS
        # default paremeter the EKF take https://docs.px4.io/master/en/advanced_config/parameter_reference.html#EKF2_REQ_EPH
        # more info https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html
        self.h.stamp = rospy.Time.now()
        lat1 = self.gps_out_lat
        lon1 = self.gps_out_lon
        alt1 = self.gps_out_alt
        geo1 = GeoPoint(latitude=lat1,longitude=lon1,altitude=alt1)
        eph1 = 1#3 # 65535 = uknown #1
        epv1 = 1#5 # 65535 = uknown #1
        vel1 = int(self.gps_vel*mToCm) #  cm/s
        vn1 = int(self.speed_y*mToCm)
        ve1 = int(self.speed_x*mToCm)
        vd1 = int(1.0*mToCm)
        cog1 = 65535# int(self.cog_drone_current) # if it's 65535 it's unknown
        sat_no = 12


        print('Normal lat= ',self.gps_out_lat)
        print('Normal lon= ',self.gps_out_lon)
        print('Normal alt= ',self.gps_out_alt)
        print ''

        self.pub_spoofing_signal.publish(vd=vd1,ve=ve1,vn=vn1,header=self.h,fix_type=3,geo=geo1,eph=eph1,epv=epv1,vel=vel1,cog=cog1,satellites_visible=sat_no)
        self.count +=1
        print("data PUB ",self.count)



if __name__ == '__main__':

    try:
        SpoofingClass()
    except rospy.ROSInterruptException as e:
        print (e)
