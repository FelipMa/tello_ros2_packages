from tello_interface import Tello

tello = Tello()
tello.connect()
print(f"battery: {tello.get_battery()}")
print(f"battery queried: {tello.query_battery()}")
