from tello_interface import Tello
import time

tello = Tello()
tello.connect()
print(f"battery: {tello.get_battery()}")

print("taking off")
tello.takeoff()

print("wainting 3 seconds for takeoff")
time.sleep(3)

print("sending go_xyz_speed(0, 50, 0, 35)")
tello.go_xyz_speed(0, 50, 0, 35)

print("wainting 3 seconds for moving")
time.sleep(3)

print("landing")
tello.land()

print("wainting 3 seconds for landing")
time.sleep(3)