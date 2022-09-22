#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, LocationLocal
import math
from pymavlink import mavutil

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線

def relay(Instance,setting):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY, #command
        0, #confirmation
        Instance,setting,0,0,0,0,0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def aux(ACTUATOR,pwm):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        ACTUATOR,pwm,0,0,0,0,0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

# for _ in range(3):
#     aux(12,900)
#     print(" Ch12:900" )
#     time.sleep(1)
#     aux(12,1900)
#     print(" Ch12:1900")
#     time.sleep(1)

relay(0,1)
print("OPEN")
time.sleep(3)
aux(12,900)
relay(0,0)
print("CLOSE")
time.sleep(1)
aux(12,1900)



print("Close vehicle object")
vehicle.close()
