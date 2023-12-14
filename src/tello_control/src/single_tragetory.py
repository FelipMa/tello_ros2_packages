#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np
import threading
from typing import List

class SingleTragetory(Node):

    def __init__(self):
        super().__init__('SingleTragetory')

        self.control_pub = self.create_publisher(Twist, 'control', 1)
        self.sub_pos = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)

        self.pos = None

        self.run()

    def odom_callback(self, msg: Odometry):
        print("odom_callback")
        self.pos = msg.pose.pose.position
        print(self.pos)
        time.sleep(2)

    def run(self):
        while rclpy.ok():
            print("run")
            time.sleep(5)
    
def main(args=None):
    rclpy.init(args=args)

    node = SingleTragetory()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()