#!/usr/bin/env python3

import time
from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import numpy as np

class tello_controller():

    def __init__(self, node: Node, ns: str):
        self.node = node

        self.pub_takeoff = node.create_publisher(Empty, ns + '/takeoff', 1)
        self.pub_land = node.create_publisher(Empty, ns + '/land', 1)
        self.pub_cmd_vel = node.create_publisher(Twist, ns + '/cmd_vel', 1)
        
        self.sub_pos = node.create_subscription(Odometry, ns + '/odom', self.odom_callback, 1)

        self.max_velocity = 0.03

        self.pos = None
        self.consensus_agreed_state = None

        self.at_target = False

    def check_subscribers(self, pub: Publisher, timeout: float = 1.0) -> bool:
        counter = 0
        while True:
            connections = pub.get_subscription_count()
            if connections == 1:
                return True
            elif connections > 1:
                self.node.get_logger().info('Multiple subscribers connected to ' + pub.topic_name + ' (' + str(connections) + ')')
                return True
            else:
                if counter >= timeout:
                    self.node.get_logger().info('No subscribers connected to ' + pub.topic_name)
                    return False
                else:
                    counter += 0.05
                    time.sleep(0.05)
        
    def takeoff(self) -> None:
        cmd = Empty()
        if self.check_subscribers(self.pub_takeoff):
            self.pub_takeoff.publish(cmd)

    def land(self) -> None:
        cmd = Empty()
        if self.check_subscribers(self.pub_land):
            self.pub_land.publish(cmd)

    def publish_velocity(self, cmd: Twist) -> None:
        if self.check_subscribers(self.pub_cmd_vel):
            self.pub_cmd_vel.publish(cmd)

    def odom_callback(self, msg: Odometry) -> None:
        self.pos = msg.pose.pose.position

    def get_pos(self) -> Point:
        return self.pos
    
    def get_consensus_agreed_state(self) -> Point:
        return self.consensus_agreed_state
    
    def set_consensus_agreed_state(self, pos: Point) -> None:
        self.consensus_agreed_state = pos
    
    def move_to_pos(self, target_pos: Point) -> None:
        msg = Twist()

        if self.at_target:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
        else:
            msg.linear.x = np.clip(target_pos.x - self.pos.x, -self.max_velocity, self.max_velocity)
            msg.linear.y = np.clip(target_pos.y - self.pos.y, -self.max_velocity, self.max_velocity)

        self.publish_velocity(msg)


