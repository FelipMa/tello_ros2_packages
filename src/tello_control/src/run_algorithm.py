#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import numpy as np
import threading
from typing import List
import copy

class control_algorithm():

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('control_algorithm')
        self.drones: List[tello_controller] = []
        self.number_of_drones = 4
        self.finish = False
        self.distance_threshold = 0.05
        self.max_velocity = 0.03

        # Set up the reference position to be reached
        posRef = Point()
        posRef.x = 1.5
        posRef.y = 1.0
        self.posRef = posRef

        # Create tello controllers
        for i in range(1, self.number_of_drones + 1):
            ns = '/simu_tello' + str(i)
            self.drones.append(tello_controller(self.node, ns))

        # Spin the node in a separate thread
        spin_node_thread = threading.Thread(target=rclpy.spin, args=(self.node,))
        spin_node_thread.start()

        # Take off
        for drone in self.drones:
            drone.takeoff()
        time.sleep(4)

        # Change altitudes
        self.getUp()
        time.sleep(2)

        # Wait for the drones to get up and start publishing their positions
        while True:
            if all([drone.get_pos() is not None for drone in self.drones]):
                break
            else:
                time.sleep(0.05)

        # Start the consensus algorithm
        while rclpy.ok():
            self.consensus_algorithm()
            if self.finish:
                self.stop()
                print("Finished!")
                break

        rclpy.shutdown()

    def getUp(self):
        msg = Twist()

        for i in range(0, self.number_of_drones):
            msg.linear.z = 0.05 * (i+1)
            self.drones[i].publish_velocity(msg)

        time.sleep(1)

        for i in range(0, self.number_of_drones):
            msg.linear.z = 0.0
            self.drones[i].publish_velocity(msg)

    def stop(self):
        msg = Twist()

        for i in range(0, self.number_of_drones):
            self.drones[i].publish_velocity(msg)

    def seeOtherDrones(self, dronePos: Point, otherPos: List[Point]) -> List[int]:
        radius = 3
        seeArray = [0] * len(otherPos)

        for i in range(0, len(otherPos)):
            if (dronePos.x - otherPos[i].x)**2 + (dronePos.y - otherPos[i].y)**2 <= radius**2:
                seeArray[i] = 1
            else:
                seeArray[i] = 0

        return seeArray
    
    def control_law(self, positions_matrix: List[Point], comunicate_matrix: List[List[int]]) -> List[Point]:
        next_positions = copy.deepcopy(positions_matrix)

        for i in range(0, len(positions_matrix)):
            ctrl_law_Ux = 0.0
            ctrl_law_Uy = 0.0
            comunications = 1

            for j in range(0, len(positions_matrix)):
                if comunicate_matrix[i][j]:
                    comunications += 1
                    
                    temp_Ux = comunicate_matrix[i][j] * (positions_matrix[i].x - positions_matrix[j].x)
                    ctrl_law_Ux = ctrl_law_Ux - temp_Ux

                    temp_Uy = comunicate_matrix[i][j] * (positions_matrix[i].y - positions_matrix[j].y)
                    ctrl_law_Uy = ctrl_law_Uy - temp_Uy

            next_positions[i].x = ctrl_law_Ux / comunications
            next_positions[i].y = ctrl_law_Uy / comunications

        return next_positions
    
    # to do, check if the graph is connected
    def missionCanBeCompleted(self, comunicate_matrix: List[List[int]]):
        return True
    
    def consensus_algorithm(self):
    
        pos_matrix: List[Point] = [drone.get_pos() for drone in self.drones]
        pos_matrix.append(self.posRef)

        visions: List[List[int]] = []
        for i in range(0, self.number_of_drones):
            visions.append(self.seeOtherDrones(pos_matrix[i], [pos_matrix[j] for j in range(0, len(pos_matrix)) if j != i]))

        comunicate_matrix: List[List[int]] = []
        for i in range(0, self.number_of_drones):
            comunicate_matrix.append(visions[i])
            comunicate_matrix[i].insert(i, 0)

        ref_vis = [0] * (self.number_of_drones+1)
        comunicate_matrix.append(ref_vis)

        if self.missionCanBeCompleted(comunicate_matrix):
            pass
        else:
            print("Mission can not be completed!")
            self.finish = True
            return
        
        next_positions: List[Point] = self.control_law(pos_matrix, comunicate_matrix)
        
        move_msg = Twist()

        finishArray = [0] * self.number_of_drones

        for i in range(0, self.number_of_drones):
            law_x = next_positions[i].x
            law_y = next_positions[i].y

            if abs(law_x) < self.distance_threshold:
                move_msg.linear.x = 0.0
            else:
                move_msg.linear.x = np.clip(law_x, -self.max_velocity, self.max_velocity)

            if abs(law_y) < self.distance_threshold:
                move_msg.linear.y = 0.0
            else:
                move_msg.linear.y = np.clip(law_y, -self.max_velocity, self.max_velocity)

            self.drones[i].publish_velocity(move_msg)

            if move_msg.linear.x == 0.0 and move_msg.linear.y == 0.0:
                finishArray[i] = 1

        if sum(finishArray) == self.number_of_drones:
            self.finish = True

class tello_controller():

    def __init__(self, node: Node, ns: str):
        self.node = node

        self.pub_takeoff = node.create_publisher(Empty, ns + '/takeoff', 1)
        self.pub_land = node.create_publisher(Empty, ns + '/land', 1)
        self.pub_cmd_vel = node.create_publisher(Twist, ns + '/cmd_vel', 1)
        
        self.sub_pos = node.create_subscription(Odometry, ns + '/odom', self.odom_callback, 1)

        self.pos = None

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

if __name__ == '__main__':
    control_algorithm()
