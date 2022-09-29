from dronekit import connect
from pymavlink import mavutil
import time

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200)

def follow(altitude):
    msg = vehicle.message_factory.command_long_encode(
        1, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_FOLLOW, #command
        1,0,0,1,altitude,0,0,0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()
while True:
    follow(20)
    time.sleep(2)
vehicle.close()