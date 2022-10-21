from dronekit import connect
from utils.drone import arm_and_takeoff, aux, relay

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200)

arm_and_takeoff(vehicle,10)


vehicle.close()