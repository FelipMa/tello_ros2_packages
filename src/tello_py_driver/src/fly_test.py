from tello_interface import Tello
import time

tello = Tello()
tello.connect()
print(f"battery: {tello.get_battery()}")

print("taking off")
tello.takeoff()

print("wainting 3 seconds for takeoff")
time.sleep(3)

print("setting speed to 35cm/s")
tello.set_speed(35)

print("wainting 1 seconds for setting speed")
time.sleep(1)

print("moving forward 50cm")
tello.move_right(50)

print("wainting 3 seconds for moving")
time.sleep(3)

print("landing")
tello.land()

print("wainting 3 seconds for landing")
time.sleep(3)