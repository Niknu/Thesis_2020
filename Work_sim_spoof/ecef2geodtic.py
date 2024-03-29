#!/usr/bin/env python

from matplotlib import pyplot as plt
from math import *
from ecef2geodtic_def import *


#------copied from the paper(https://hal.archives-ouvertes.fr/hal-01704943v2/document) 

class ecefGeo_class :
    
    def __init__(self):
        self.alt_0=0
        self.First_run = True

    def AddOffset_xy(self,x,y,dt,speed_x,speed_y,same_speed_xy=False):


        if same_speed_xy is True:
            off_x = x+(dt*speed_x)
            off_y = y+(dt*speed_x)
        else:
            off_x = x+(dt*speed_x)
            off_y = y+(dt*speed_y)


        return off_x, off_y

    def GeoToEcef(self,lat_in,lon_in,alt_in):

        lat = lat_in
        lon = lon_in
        alt = alt_in

        if self.First_run is True:
            self.alt_0 = alt_in
            self.First_run = False


        lat = d2r * lat
        lon = d2r * lon
        alt = d2r * alt

        coslat = cos(lat)
        sinlat = sin(lat)
        coslon = cos(lon)
        sinlon = sin(lon)

        N = aadc / sqrt(coslat * coslat + bbdcc)
        d = (N + alt) * coslat

        x = d * coslon
        y = d * sinlon
        z = (p1mee * N + alt) * sinlat

        return x,y,z

    def Ecef2Geo(self,x,y,z):

        
        '''
        print(' -- Start pos for xyz')
        print('x=',x)
        print('y=',y)
        print('z=',z)
        '''

        ww = x * x + y * y
        m = ww * invaa
        n = z * z * p1meedaa
        mpn = m + n
        p = inv6 * (mpn - ll4)
        G = m * n * ll
        H = 2.0 * p * p * p + G


        if (H < Hmin):
            print('something is wrong With H')
            #return -1


        C = ((H + G + 2.0 * sqrt(H * G))**(1.0/3.0)) * invcbrt2
        i = -ll - 0.5 * mpn
        P = p * p
        beta = inv3 * i - C - P / C
        k = ll * (ll - mpn)

        # Compute left part of t
        t1 = beta * beta - k
        t2 = sqrt(t1)
        t3 = t2 - 0.5 * (beta + i)
        t4 = sqrt(t3)
        #Compute right part of t
        t5 = 0.5 * (beta - i)
        # t5 may accidentally drop just below zero due to numeric turbulence
        # This only occurs at latitudes close to +- 45.3 degrees
        t5 = fabs(t5)
        t6 = sqrt(t5)
        #t7 = (m < n) ? t6 : -t6
        t7 = t6 if (m<n) else -t6
        # Add left and right parts
        t = t4 + t7

        #Use Newton-Raphson's method to compute t correction
        j = l * (m - n)
        g = 2.0 * j
        tt = t * t
        ttt = tt * t
        tttt = tt * tt
        F = tttt + 2.0 * i * tt + g * t + k
        dFdt = 4.0 * ttt + 4.0 * i * t + g
        dt = -F / dFdt

        #compute latitude (range -PI/2..PI/2)
        u = t + dt + l
        v = t + dt - l
        w = sqrt(ww)
        zu = z * u
        wv = w * v
        lat = atan2(zu, wv)

        #compute altitude
        invuv = 1.0 / (u * v)
        dw = w - wv * invuv
        dz = z - zu * p1mee * invuv
        da = sqrt(dw * dw + dz * dz)
        #alt = (u < 1) ? -da : da
        alt = -da if (u < 1.0) else da

        #compute longitude (range -PI..PI)
        lon = atan2(y, x)


        #Converte it to degrees
        lat = r2d*lat
        lon = r2d*lon
        alt = r2d*alt

        if self.alt_0 <= alt+37.0:  #This is implemented to removing the driffing part when it's stable hovering (37.0 is a guess)
            alt = self.alt_0
            

        return lat, lon, alt



if __name__ == "__main__":

    kll= ecefGeo_class()
    # start position
    lat = 55.4719841
    lon = 10.3248241
    alt = 41.1763002681 #535.314901042  #

    data_x=[]
    data_y=[]
    data_z=[]
    data_time=[]
    data_diff_z=[]
    data_diff_z_time=[]
    i=0
    #data_x.append(x)
    #data_y.append(y)
    data_z.append(alt)
    data_time.append(i)

    for i in range(1000):
        i=i+1
        
        x,y,z = kll.GeoToEcef(lat,lon,alt)
        x,y = kll.AddOffset_xy(x,y,1.0,1.0,1.0)
        data_x.append(x)
        data_y.append(y)
        lat,lon,alt = kll.Ecef2Geo(x,y,z)
        data_z.append(alt)
        data_time.append(i)

        if i > 0:
            data_diff_z_time.append(i)
            x0 = data_z[i-1]
            x1 = data_z[i]

        #print 'diff in alt1-alt0=', x1-x0
        data_diff_z.append(x1-x0)

    '''
    print('')
    print(' -- Output from the function')
    print('latitude = ',lat)
    print('longitude = ',lon)
    print('altitude = ',alt)
    '''

    #plt.scatter(data_x,data_y,label='Spoofing data')#,color='red',linestyle='--')
    plt.scatter(data_time,data_z,label='Increase in alt when its stable')#,color='red',linestyle='--')
    plt.legend(loc='upper right')
    plt.grid(False)
    plt.show()

    plt.plot(data_diff_z,label='difference between z0 and z1')#,color='red',linestyle='--')
    plt.legend(loc='upper left')
    plt.grid(False)
    plt.show()