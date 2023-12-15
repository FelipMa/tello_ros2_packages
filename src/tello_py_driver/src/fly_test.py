from tello_interface import Tello
import time

tello = Tello()
tello.connect()
print(f"battery: {tello.get_battery()}")

tello.takeoff()

print("wainting 5 seconds for takeoff")
time.sleep(5)

tello.send_rc_control(35, 0, 0, 0)

print("wainting 3 seconds for moving")
time.sleep(3)

tello.send_rc_control(0, 0, 0, 0)

print("wainting 1 seconds for stopping")
time.sleep(1)

tello.send_rc_control(-35, 0, 0, 0)

print("wainting 3 seconds for moving")
time.sleep(3)

tello.send_rc_control(0, 0, 0, 0)

print("wainting 1 seconds for stopping")
time.sleep(1)

tello.land()