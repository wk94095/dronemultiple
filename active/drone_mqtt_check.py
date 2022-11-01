#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal
import math
from pymavlink import mavutil
import utils
import getch
import paho.mqtt.client as mqtt
import json

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線
first_vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=115200) #與飛機連線

def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def target(dis, dhead, dalt):
    lat = first_vehicle.location.global_relative_frame.lat #無人機緯度座標
    lon = first_vehicle.location.global_relative_frame.lon #無人機經度座標
    alt = dalt
    #print(vehicle.attitude.pitch) #顯示無人機pitch角度
    droneheading = math.radians(90-first_vehicle.heading-dhead) #飛機頭向
    distance = dis
    #print("離目標物距離:",distance)
    earth_radius=6378137.0 #地球半徑
    lat1 = distance*math.sin(droneheading)
    lon1 = distance*math.cos(droneheading)
    #print(lat1,lon1)
    dlat = lat1/earth_radius
    dlon = lon1/(earth_radius*math.cos(math.pi*lat/180))

    newlat = lat + (dlat * 180/math.pi)
    newlon = lon + (dlon * 180/math.pi)
    #print("new",newlat,newlon)

    # distancelat = newlat - lat
    # distancelon = newlon - lon
    # get_distance_metres = math.sqrt((distancelat**2)+(distancelon**2))* 1.113195e5
    #print("座標離目標物距離:",get_distance_metres)

    return LocationGlobalRelative(newlat, newlon, alt)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # 將訂閱主題寫在on_connet中
    # 如果我們失去連線或重新連線時 
    # 地端程式將會重新訂閱
    client.subscribe("drone/mode")

# 當接收到從伺服器發送的訊息時要進行的動作
def on_message(client, userdata, msg):
    # 轉換編碼utf-8才看得懂中文
    print(msg.topic+" "+ msg.payload.decode('utf-8'))
    data = json.loads(msg.payload.decode('utf-8'))
    temp = data["mode"]
    firstalt = first_vehicle.location.global_relative_frame.alt
    if temp == 'line':
        start = time.time()
        point = target(10, -180,firstalt-2) #(距離，與長機相對位置， 高度)
        vehicle.simple_goto(point,6)
        time.sleep(1)
        distancetopoint = utils.get_distance_metres(vehicle.location.global_frame, point)
        if distancetopoint >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
            #vehicle.simple_goto(point1)
            print("Distance to target:"+"{:.2f}".format(distancetopoint)) #{}內容會讀取後面.format內的值，如{:.3f}表示將remainingDistance填充到槽中時，取小數點後3位
            if first_vehicle.mode == "RTL":
                print("Returning to Launch")
                vehicle.mode = VehicleMode("LAND")
                #break
                # vehicle.simple_goto(home)
                # if vehicle.location.global_relative_frame == home:
                #     vehicle.mode = VehicleMode("LAND")
                #     home =0
                #     break
        #elif ord(getch.getch()) in [81,113]:
        elif first_vehicle.mode == "RTL":
            print("Returning to Launch")
            vehicle.mode = VehicleMode("LAND")
            #break
            # vehicle.simple_goto(home)
            # dist = utils.get_distance_metres(vehicle.location.global_relative_frame, home)
            # print(home)
            # print(dist)
            # if dist*0.8 <= 1:
            #     vehicle.mode = VehicleMode("LAND")
            #     home =0
            #     break
                
        # elif first_vehicle.heading != vehicle.heading:
        #     condition_yaw(first_vehicle.heading)

        elif distancetopoint*0.95<=1:
            print ("Reached target")

        else:
            print("Change Mode Guided")
            vehicle.mode = VehicleMode("GUIDED")
        end = time.time()
        #print("執行時間:"+ str(end-start))
    elif temp == 'triangle':
        start = time.time()
        point = target(10, -225,firstalt-2)
        vehicle.simple_goto(point,6)
        time.sleep(1)
        distancetopoint = utils.get_distance_metres(vehicle.location.global_frame, point)
        if distancetopoint >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
            #vehicle.simple_goto(point1)
            print("Distance to target:"+"{:.2f}".format(distancetopoint)) #{}內容會讀取後面.format內的值，如{:.3f}表示將remainingDistance填充到槽中時，取小數點後3位
            if first_vehicle.mode == "RTL":
                print("Returning to Launch")
                vehicle.mode = VehicleMode("LAND")
                #break
                # vehicle.simple_goto(home)
                # if vehicle.location.global_relative_frame == home:
                #     vehicle.mode = VehicleMode("LAND")
                #     home =0
                #     break
        #elif ord(getch.getch()) in [81,113]:
        elif first_vehicle.mode == "RTL":
            print("Returning to Launch")
            vehicle.mode = VehicleMode("LAND")
            #break
            # vehicle.simple_goto(home)
            # dist = utils.get_distance_metres(vehicle.location.global_relative_frame, home)
            # print(home)
            # print(dist)
            # if dist*0.8 <= 1:
            #     vehicle.mode = VehicleMode("LAND")
            #     home =0
            #     break
                
        # elif first_vehicle.heading != vehicle.heading:
        #     condition_yaw(first_vehicle.heading)

        elif distancetopoint*0.95<=1:
            print ("Reached target")

        else:
            print("Change Mode Guided")
            vehicle.mode = VehicleMode("GUIDED")
        end = time.time()
        #print("執行時間:"+ str(end-start))
    elif temp == 'Inverted triangle':
        start = time.time()
        point = target(10, -315,firstalt-2)
        vehicle.simple_goto(point,6)
        time.sleep(1)
        distancetopoint = utils.get_distance_metres(vehicle.location.global_frame, point)
        if distancetopoint >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
            #vehicle.simple_goto(point1)
            print("Distance to target:"+"{:.2f}".format(distancetopoint)) #{}內容會讀取後面.format內的值，如{:.3f}表示將remainingDistance填充到槽中時，取小數點後3位
            if first_vehicle.mode == "RTL":
                print("Returning to Launch")
                vehicle.mode = VehicleMode("LAND")
                #break
                # vehicle.simple_goto(home)
                # if vehicle.location.global_relative_frame == home:
                #     vehicle.mode = VehicleMode("LAND")
                #     home =0
                #     break
        #elif ord(getch.getch()) in [81,113]:
        elif first_vehicle.mode == "RTL":
            print("Returning to Launch")
            vehicle.mode = VehicleMode("LAND")
            #break
            # vehicle.simple_goto(home)
            # dist = utils.get_distance_metres(vehicle.location.global_relative_frame, home)
            # print(home)
            # print(dist)
            # if dist*0.8 <= 1:
            #     vehicle.mode = VehicleMode("LAND")
            #     home =0
            #     break
                
        # elif first_vehicle.heading != vehicle.heading:
        #     condition_yaw(first_vehicle.heading)

        elif distancetopoint*0.95<=1:
            print ("Reached target")

        else:
            print("Change Mode Guided")
            vehicle.mode = VehicleMode("GUIDED")
        end = time.time()
        #print("執行時間:"+ str(end-start))

if vehicle.armed != True:
    utils.arm_and_takeoff(first_vehicle,vehicle,10) #起飛高度
    print("takeoff")
    
else:
    vehicle.mode = VehicleMode("GUIDED")
    print("change mode")
# 連線設定
# 初始化地端程式
client = mqtt.Client()

# 設定連線的動作
client.on_connect = on_connect

# 設定接收訊息的動作
client.on_message = on_message

# 設定登入帳號密碼
client.username_pw_set("bighead1","nfuaesil1")

# 設定連線資訊(IP, Port, 連線時間)
client.connect("192.168.0.132", 1883, 60)


# print("Set default/target airspeed to 8")
# vehicle.airspeed = 8
#home = vehicle.location.global_relative_frame
#while True:
    
client.loop_forever()
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
first_vehicle.close()
