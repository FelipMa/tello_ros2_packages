#!/usr/bin/env python3

import rclpy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from tello_msgs.msg import TelloStatus
import numpy as np
from sensor_msgs.msg import BatteryState, Image

import cv2


class TelloKeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('ns', 'keyboard_teleop')
        self.window_name = self.get_parameter('ns').get_parameter_value().string_value

        self.publisher_land = self.create_publisher(Empty, 'land', 1)
        self.publisher_flip = self.create_publisher(String, 'flip', 1)
        self.publisher_takeoff = self.create_publisher(Empty, 'takeoff', 1)
        self.publisher_velocity = self.create_publisher(Twist, 'control', 1)
        self.publisher_emergency = self.create_publisher(Empty, 'emergency', 1)
        self.sub_battery = self.create_subscription(BatteryState, 'battery', self.battery_callback, 1)
        self.sub_image = self.create_subscription(Image, 'image_raw', self.video_receiver_callback, 1)
        self.sub_status = self.create_subscription(TelloStatus, "status", self.status_callback, 1)
        self.pub_cmd_vel_1 = self.create_publisher(
            Twist, '/simu_tello1/cmd_vel', 1)

        self.battery = None
        self.status = None
        self.manual_speed = 35.0  # cm/s

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.last_key = -1

        self.frame = np.zeros((400, 400, 3), np.uint8)

    def manual_control(self, key):
        msg = Twist()

        if key == 81:  # left arrow
            msg.angular.z = -self.manual_speed
            self.get_logger().info("left arrow")
        elif key == 83:  # right arrow
            msg.angular.z = self.manual_speed
            self.get_logger().info("right arrow")
        elif key == 82:  # up arrow
            msg.linear.z = self.manual_speed
            self.get_logger().info("up arrow")
        elif key == 84:  # down arrow
            msg.linear.z = -self.manual_speed
            self.get_logger().info("down arrow")
        elif key == ord('a'):
            msg.linear.y = -self.manual_speed
            self.get_logger().info("left")
        elif key == ord('d'):
            msg.linear.y = self.manual_speed
            self.get_logger().info("right")
        elif key == ord('w'):
            msg.linear.x = self.manual_speed
            self.get_logger().info("forward")
        elif key == ord('s'):
            msg.linear.x = -self.manual_speed
            self.get_logger().info("backward")
        else:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = 0.0
            self.get_logger().info(f"key {key} not mapped, stopping")

        self.publisher_velocity.publish(msg)
        
        simu_msg = Twist()
        simu_msg.linear.x = msg.linear.x / 1000
        simu_msg.linear.y = msg.linear.y / 1000
        simu_msg.linear.z = msg.linear.z / 1000
        simu_msg.angular.z = msg.angular.z / 1000
        self.pub_cmd_vel_1.publish(simu_msg)

    def timer_callback(self):

        cv2.imshow(self.window_name, self.frame)
        key = cv2.waitKey(15) # wait key for 15 ms, if no key is pressed, return -1

        if key != -1:
            if key == ord('t'):
                self.publisher_takeoff.publish(Empty())
                self.get_logger().info("Takeoff")
            elif key == ord('l'):
                self.publisher_land.publish(Empty())
                self.get_logger().info("Land")
            elif key == ord('f'):
                msg = String()
                msg.data = 'f'
                self.publisher_flip.publish(msg)
                self.get_logger().info("Flip")
            elif key == ord('e'):
                self.publisher_emergency.publish(Empty())
                self.get_logger().info("Emergency")
            elif key == ord('b'):
                self.get_logger().info("Battery: " + str(self.battery))
            elif key == ord('p'):
                self.get_logger().info("Status: " + str(self.status))
            elif key == 27: # ESC
                self.get_logger().info("ESC - exiting")
                cv2.destroyAllWindows()
                exit(0)
            else:
                self.manual_control(key)

    def battery_callback(self, msg: BatteryState):
        self.battery = msg.percentage

    def video_receiver_callback(self, msg: Image):
        self.frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    def status_callback(self, msg: TelloStatus):
        status = {}

        status['acc_x'] = msg.acceleration.x
        status['acc_y'] = msg.acceleration.y
        status['acc_z'] = msg.acceleration.z
        status['speed_x'] = msg.speed.x
        status['speed_y'] = msg.speed.y
        status['speed_z'] = msg.speed.z
        status['barometer'] = msg.barometer
        status['distance_tof'] = msg.distance_tof
        status['height'] = msg.height
        status['battery'] = msg.battery
        status['flight_time'] = msg.fligth_time
        status['highest_temperature'] = msg.highest_temperature
        status['lowest_temperature'] = msg.lowest_temperature
        status['pitch'] = msg.pitch
        status['roll'] = msg.roll
        status['yaw'] = msg.yaw

        self.status = status

def main(args=None):
    rclpy.init(args=args)
    node = TelloKeyboardTeleop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
