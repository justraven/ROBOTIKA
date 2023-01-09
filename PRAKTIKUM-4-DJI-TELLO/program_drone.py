from time import sleep
from djitellopy import Tello

tello = Tello()

tello.connect()
tello.get_battery()
tello.takeoff()

print("battery : ", tello.get_battery()," accel X : ", tello.get_acceleration_x(), " \n")

tello.send_rc_control(1,0,0,0)
sleep(3)

tello.land()