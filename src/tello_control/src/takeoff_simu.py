#!/usr/bin/env python3

import time
import rclpy
from std_msgs.msg import Empty
import threading


class takeoff_simu():

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('takeoff_simu')
        self.number_of_drones = 4
        self.pos1 = None

        self.pub_takeoff_1 = self.node.create_publisher(
            Empty, '/simu_tello1/takeoff', 1)
        self.pub_takeoff_2 = self.node.create_publisher(
            Empty, '/simu_tello2/takeoff', 1)
        self.pub_takeoff_3 = self.node.create_publisher(
            Empty, '/simu_tello3/takeoff', 1)
        self.pub_takeoff_4 = self.node.create_publisher(
            Empty, '/simu_tello4/takeoff', 1)

        spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,))
        spin_thread.start()

        time.sleep(0.5)

        self.takeoff_all()

        rclpy.shutdown()

    def publish_once_in_cmd_vel(self, pubid: int, cmd: Empty):
        if pubid == 1:
            pub = self.pub_takeoff_1
        elif pubid == 2:
            pub = self.pub_takeoff_2
        elif pubid == 3:
            pub = self.pub_takeoff_3
        elif pubid == 4:
            pub = self.pub_takeoff_4
        else:
            return

        while True:
            connections = pub.get_subscription_count()
            if connections > 0:
                pub.publish(cmd)
                break
            else:
                time.sleep(0.005)

    def takeoff_all(self):
        cmd = Empty()
        for i in range(1, self.number_of_drones+1):
            self.publish_once_in_cmd_vel(i, cmd)


if __name__ == '__main__':
    takeoff_simu()
