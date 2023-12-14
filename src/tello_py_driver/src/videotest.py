from tello_interface import Tello
import time
import cv2

tello = Tello()
tello.connect()
print(f"battery: {tello.get_battery()}")

tello.streamon()
print("streamon")

while True:
    try:
        frame = tello.get_frame_read()

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except:
        print("no frame yet")
        time.sleep(1)

