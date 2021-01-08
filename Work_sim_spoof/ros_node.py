# -*- coding: utf-8 -*-
#!/usr/bin/env python

from ecef2geodtic import *  # 1st line is to run this file
import rospy
from sensor_msgs.msg import NavSatFix       #From the /mavros/global_position/global or mavros/global_position/raw/fix
from geographic_msgs.msg import GeoPoint    #Part of the HIL_GPS
from mavros_msgs.msg import HilGPS 
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue



class SpoofingClass():

    def __init__(self):
        self.GEO_class = ecefGeo_class()
        self.t_prev = 0.0
        self.t_current = 0.0
        self.dt = 0
        self.data = None
        self.FirstRun_dt = True
        self.Pub_data = False
        self.FirstRun_pos = True
        self.FirstRun_pub = True

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


        #init the node and subscribe to the raw signal from the gps
        rospy.init_node('get_GPS_data',anonymous=True)

        #### -------- Check if mavros/global_position/raw/fix gives better results ----- ########
        rospy.Subscriber('/mavros/global_position/global',NavSatFix,self.load_data) #The '/mavros/global_position/global' used also the data from the IMU (http://wildfirewatch.elo.utfsm.cl/ros-topology/#global_gps)
        
        #Create publisher to the HIL gps to send spoofing signal
        self.pub_spoofing_signal = rospy.Publisher('/mavros/hil/gps',HilGPS,queue_size=10) # remember to 'param set MAV_USEHILGPS 1' in the px4 before this can work

        #self.pub_param_set = rospy.Publisher('/mavros/param/param_value',ParamSet,queue_size=1)

        rospy.Rate(10) # 1000 Hz
        
        rospy.spin()
        

    def load_data(self,data):
        self.data = data

        self.get_dt()

        self.transform_gps_xyz()

        self.publish_data()

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
        
        # -- Get the data
        #self.get_dt()


        #This is done so we have the 
        if self.FirstRun_pos is True:
            self.get_pos()
            self.FirstRun_pos = False
            #Transform Geo 2 ECEF
            self.ecef_x,self.ecef_y,self.ecef_z = self.GEO_class.GeoToEcef(self.gps_in_lat,self.gps_in_lon,self.gps_in_alt)

        speed_x = 0.0  #[m/s]
        speed_y = 1.0 #[m/s]

        #Add offset
        x , y = self.GEO_class.AddOffset_xy(self.ecef_x,self.ecef_y,self.dt,speed_x,speed_y)
        self.ecef_x = x
        self.ecef_y = y
        
        #Transform ECEF 2 Geo
        lat, lon, alt = self.GEO_class.Ecef2Geo(self.ecef_x,self.ecef_y,self.ecef_z)
        self.gps_out_lat = lat
        self.gps_out_lon = lon
        self.gps_out_alt = alt

        print (self.gps_out_lat, self.gps_out_lon, self.gps_out_alt)


    '''
    def publish_param(self):
        val = ParamValue(integer=1)
        self.pub_param_set.publish(param_id='SIM_GPS_BLOCK',value=val)
    '''
    def publish_data(self):
        

        '''
        if self.FirstRun_pub is True:
            self.publish_param()
            self.FirstRun_pub = False
        '''

        lat1 = self.gps_out_lat
        lon1 = self.gps_out_lon 
        alt1 = self.gps_out_alt
        geo1 = GeoPoint(latitude=lat1,longitude=lon1,altitude=alt1)
        eph1 = 1
        epv1 = 1
        vel1 = 0
        cog1 = 0
        sat_no = 12
        self.pub_spoofing_signal.publish(fix_type=3,geo=geo1,eph=eph1,epv=epv1,vel=vel1,cog=cog1,satellites_visible=sat_no)




if __name__ == '__main__':
    try:
        SpoofingClass()
    except rospy.ROSInterruptException as e:
        print (e)
