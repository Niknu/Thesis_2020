#!/usr/bin/env python

from math import *
from ecef2geodtic_def import *


#------copied from the paper(https://hal.archives-ouvertes.fr/hal-01704943v2/document) 

def GeoToEcef(lat_in,lon_in,alt_in):

    lat = lat_in
    lon = lon_in
    alt = alt_in

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

def Ecef2Geo(lat_in,lon_in,alt_in):

    x,y,z = GeoToEcef(lat_in,lon_in,alt_in)

    print(' -- Start pos for xyz')
    print('x=',x)
    print('y=',y)
    print('z=',z)





    ww = x * x + y * y
    m = ww * invaa
    n = z * z * p1meedaa
    mpn = m + n
    p = inv6 * (mpn - ll4)
    G = m * n * ll
    H = 2 * p * p * p + G


    if (H < Hmin):
        print('something is wrong With H')
        #return -1;


    C = ((H + G + 2 * sqrt(H * G))**(1/3)) * invcbrt2
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
    t5 = 0.5 * (beta - i);
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
    g = 2 * j
    tt = t * t
    ttt = tt * t
    tttt = tt * tt
    F = tttt + 2 * i * tt + g * t + k
    dFdt = 4 * ttt + 4 * i * t + g
    dt = -F / dFdt

    #compute latitude (range -PI/2..PI/2)
    u = t + dt + l
    v = t + dt - l
    w = sqrt(ww)
    zu = z * u
    wv = w * v
    lat = atan2(zu, wv)

    #compute altitude
    invuv = 1 / (u * v)
    dw = w - wv * invuv
    dz = z - zu * p1mee * invuv
    da = sqrt(dw * dw + dz * dz)
    #alt = (u < 1) ? -da : da;
    alt = -da if (u < 1) else da

    #compute longitude (range -PI..PI)
    lon = atan2(y, x)


    #Converte it to degrees
    lat = r2d*lat
    lon = r2d*lon
    alt = r2d*alt

    return lat, lon, alt




if __name__ == "__main__":

    # start position
    lat = 55.4719841
    lon = 10.3248241
    alt = 41.1763002681

    lat,lon,alt = Ecef2Geo(lat,lon,alt)

    print('')
    print(' -- Output from the function')
    print('latitude = ',lat)
    print('longitude = ',lon)
    print('altitude = ',alt)
