# -*- coding: utf-8 -*-
#!/usr/bin/env python

from ecef2geodtic import *  # 1st line is to run this file
import rospy
from sensor_msgs.msg import NavSatFix       #From the /mavros/global_position/global or mavros/global_position/raw/fix
from geographic_msgs.msg import GeoPoint    #Part of the HIL_GPS
from mavros_msgs.msg import HilGPS 



class SpoofingClass():

    def __init__(self):
        self.t_prev = 0.0
        self.t_current = 0.0
        self.dt = 0
        self.data = None
        self.FirstRun_dt = True

        #The data from the raw gps signal
        self.gps_in_lat = 0.0
        self.gps_in_lon = 0.0
        self.gps_in_alt = 0.0

        # The xyz for ecef
        self.ecef_x = 0.0
        self.ecef_y = 0.0
        self.ecef_z = 0.0

        #The spoofing data that will be sent to the HIL_GPS
        self.gps_out_lat = 0.0
        self.gps_out_lon = 0.0
        self.gps_out_alt = 0.0


        rospy.init_node('get_GPS_data',anonymous=True)
        rospy.Subscriber('/mavros/global_position/raw/fix',NavSatFix,self.load_data) #The '/mavros/global_position/global' used also the data from the IMU (http://wildfirewatch.elo.utfsm.cl/ros-topology/#global_gps)
        
        #rospy.Publisher()
        #rospy.Rate(1000) # 1000 Hz
        
        rospy.spin()
        

    def load_data(self,data):
        self.data = data

        #self.get_dt()

        self.transform_gps_xyz()

    def get_dt(self):

        #print('The headers time secs=',self.data.header.stamp.secs)
        #print('The headers time nsecs=',self.data.header.stamp.nsecs)
        t_secs_new = self.data.header.stamp.secs
        t_nsecs_new = self.data.header.stamp.nsecs*1.0e-9            # Convert from nsecs to secs

        self.t_current = t_secs_new + t_nsecs_new
        
        if self.FirstRun_dt is not True:
            self.dt = self.t_current - self.t_prev                       #Find the time difference (delta t)

        self.t_prev = self.t_current

        self.FirstRun_dt = False

        #print'dt = ',self.dt

    def get_pos(self):
        self.gps_in_lat = self.data.latitude
        self.gps_in_lon = self.data.longitude
        self.gps_in_alt = self.data.altitude

    def transform_gps_xyz(self):
        
        #Get the data
        self.get_dt()
        self.get_pos()


        #Transform Geo 2 ECEF
        self.ecef_x,self.ecef_y,self.ecef_z = GeoToEcef(self.gps_in_lat,self.gps_in_lon,self.gps_in_alt)
        
        speed_x = 0.0  #[m/s]
        speed_y = 0.0  #[m/s]

        #Add offset
        x , y = AddOffset_xy(self.ecef_x,self.ecef_y,self.dt,speed_x,speed_y)
        self.ecef_x = x
        self.ecef_y = y
        
        #Transform ECEF 2 Geo
        lat, lon, alt = Ecef2Geo(self.ecef_x,self.ecef_y,self.ecef_z)
        self.gps_out_lat = lat
        self.gps_out_lon = lon
        self.gps_out_alt = alt

        print self.gps_out_lat, self.gps_out_lon, self.gps_out_alt






if __name__ == '__main__':
    try:
        SpoofingClass()
    except rospy.ROSInterruptException:
        pass
