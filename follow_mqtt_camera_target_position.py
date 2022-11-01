#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import math
import json
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Attitude
from utils.gimbalmath import target
from utils.drone import arm_and_takeoff, aux, relay, get_distance_metres
import paho.mqtt.client as mqtt
start = time.time()
vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線

# 當地端程式連線伺服器得到回應時，要做的動作
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # 將訂閱主題寫在on_connet中
    # 如果我們失去連線或重新連線時 
    # 地端程式將會重新訂閱
    client.subscribe("target/position")

# 當接收到從伺服器發送的訊息時要進行的動作
def on_message(client, userdata, msg):
    global target1
    # 轉換編碼utf-8才看得懂中文
    #time.sleep(0.5)
    print(msg.topic+" "+ msg.payload.decode('utf-8'))
    target1 = position(msg.payload.decode('utf-8'))
    print(target1[0])
    a = LocationGlobalRelative(target1[0], target1[1], 10)
    vehicle.simple_goto(a)

def position(data):
    data = json.loads(data)
    Lat = data["Latitude"]
    Lon = data["Longtitude"]
    #print(Lat,Lon)
    return Lat,Lon

arm_and_takeoff(vehicle, 5)

# 連線設定
# 初始化地端程式
client = mqtt.Client()

# 設定連線的動作
client.on_connect = on_connect

# 設定接收訊息的動作
client.on_message = on_message

# 設定登入帳號密碼
client.username_pw_set("bighead","nfuaesil")

# 設定連線資訊(IP, Port, 連線時間)
client.connect("192.168.0.132", 1883, 60)

end = time.time()
print("執行時間: %f 秒" %(end-start))

# 開始連線，執行設定的動作和處理重新連線問題
# 也可以手動使用其他loop函式來進行連接
client.loop_forever()
vehicle.close()