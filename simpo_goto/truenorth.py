#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, LocationLocal
import math
import getch #偵測鍵盤按鍵
import utils

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線
vehicle1 = connect('127.0.0.1:14570', wait_ready=True, baud=115200) #與飛機連線

print(vehicle.heading)
print(vehicle.location.local_frame)

def arm_and_takeoff(aTargetAltitude): #定義起飛程序
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break

        time.sleep(1)

def get_location_metres(original_location, north, east):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = north
    dLon = east
    return LocationLocal(dLat, dLon, original_location.down)

def get_distance_metres(aLocation1, aLocation2): #定義目標位置與目標位置計算出距離
    dlat = aLocation2.north - aLocation1.north
    dlong = aLocation2.east - aLocation1.east
    return math.sqrt((dlat*dlat) + (dlong*dlong))

currentLocation=vehicle1.location.local_frame
get = get_location_metres(currentLocation, 50, 20)
targetDistance=get_distance_metres(currentLocation, get)
print(targetDistance)

def true_goto(self, location, airspeed=None, groundspeed=None):
        
        if type(location, LocationLocal):
            frame = mavutil.mavlink.MAV_FRAME_LOCAL_ENU
            alt = location.down
        elif isinstance(location, LocationGlobal):
            # This should be the proper code:
            # frame = mavutil.mavlink.MAV_FRAME_GLOBAL
            # However, APM discards information about the relative frame
            # and treats any alt value as relative. So we compensate here.
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            if not self.home_location:
                self.commands.download()
                self.commands.wait_ready()
            alt = location.alt - self.home_location.alt
        else:
            raise ValueError('Expecting location to be LocationGlobal or LocationGlobalRelative.')

        self._master.mav.mission_item_send(0, 0, 0, frame,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                           0, 0, 0, location.north, location.east,
                                           alt)

        if airspeed is not None:
            self.airspeed = airspeed
        if groundspeed is not None:
            self.groundspeed = groundspeed

def goto(north, east, gotoFunction=true_goto):
    currentLocation=vehicle1.location.local_frame #當前位置
    targetLocation=get_location_metres(currentLocation, north, east)
    targetDistance=get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.local_frame, targetLocation)
        print ("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.1: #Just below target, in case of undershoot.
            print ("Reached target")
            break
        time.sleep(2)

if vehicle.armed != True:
    arm_and_takeoff(8) #起飛高度
    print("takeoff")
else:
    vehicle.mode = VehicleMode("GUIDED")
    print("change mode")

print("Set default/target airspeed to 8")
vehicle.airspeed = 8

while True:
    goto(-40, -40)
    time.sleep(2)

print("Close vehicle object")
vehicle.close()
vehicle1.close()
