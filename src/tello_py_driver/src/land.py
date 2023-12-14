from tello_interface import Tello

tello = Tello()
tello.connect()
print(f"battery: {tello.get_battery()}")
tello.land()
