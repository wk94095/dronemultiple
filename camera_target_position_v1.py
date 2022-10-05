#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Attitude

from utils.gimbalmath import target

start = time.time()
vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線

def get_location_metres(original_location, dNorth, dEast):
    
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

target(vehicle,30)
print(vehicle.location.global_relative_frame)
print(vehicle.heading)

print("====================================")

def xy(lat,lon,B0,L0):
    B = lat*math.pi/180
    L = lon*math.pi/180
    a=6378137.0
    b=6356752.3142
    e=math.sqrt(1-(b/a)**2)
    ee=math.sqrt((a/b)**2-1)
    K=(((a**2*math.cos(B0))/b)/math.sqrt(1+(ee)**2*(math.cos(B0))**2))
    X=K*(L-L0)
    Y=K*math.log(math.tan(math.pi/4+B/2)*((1-e*math.sin(B))/(1+e*math.sin(B)))**(e/2))
    return X,Y


# print(xy(23.7025595, 120.4230539, 0, 0))

vehicle.close()
end = time.time()

print("執行時間: %f 秒" %(end-start))