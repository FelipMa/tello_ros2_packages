#!/usr/bin/env python3

import time
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np
import threading
from typing import List
import copy
class control_algorithm():

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('control_algorithm')
        self.number_of_drones = 4
        self.pos1 = None
        self.pos2 = None
        self.pos3 = None
        self.pos4 = None
        self.time_counter = 0
        self.finish = False
        self.distance_threshold = 0.05

        # Set up the reference position to be reached
        posRef = Point()
        posRef.x = 1.5
        posRef.y = 1.0
        self.posRef = posRef

        # Set up publishers
        self.pub_cmd_vel_1 = self.node.create_publisher(Twist, '/simu_tello1/cmd_vel', 1)
        self.pub_cmd_vel_2 = self.node.create_publisher(Twist, '/simu_tello2/cmd_vel', 1)
        self.pub_cmd_vel_3 = self.node.create_publisher(Twist, '/simu_tello3/cmd_vel', 1)
        self.pub_cmd_vel_4 = self.node.create_publisher(Twist, '/simu_tello4/cmd_vel', 1)

        # Set up subscribers
        self.sub_pos_1 = self.node.create_subscription(Odometry, '/simu_tello1/odom', self.callback_1, 1)
        self.sub_pos_2 = self.node.create_subscription(Odometry, '/simu_tello2/odom', self.callback_2, 1)
        self.sub_pos_3 = self.node.create_subscription(Odometry, '/simu_tello3/odom', self.callback_3, 1)
        self.sub_pos_4 = self.node.create_subscription(Odometry, '/simu_tello4/odom', self.callback_4, 1)

        spin_node_thread = threading.Thread(target=rclpy.spin, args=(self.node,))
        spin_node_thread.start()

        self.getUp()
        # Wait for the drones to get up and start publishing their positions
        while self.pos1 is None or self.pos2 is None or self.pos3 is None or self.pos4 is None:
            time.sleep(0.005)

        while rclpy.ok():
            self.consensus_algorithm()
            if self.finish:
                self.stop()
                print("Finished!")
                break

        rclpy.shutdown()

    def publish_once_in_cmd_vel(self, pubid: int, cmd: Twist):
        if pubid == 1:
            pub = self.pub_cmd_vel_1
        elif pubid == 2:
            pub = self.pub_cmd_vel_2
        elif pubid == 3:
            pub = self.pub_cmd_vel_3
        elif pubid == 4:
            pub = self.pub_cmd_vel_4
        else:
            return

        while True:
            connections = pub.get_subscription_count()
            if connections > 0:
                pub.publish(cmd)
                break
            else:
                time.sleep(0.005)

    def callback_1(self, data: Odometry):
        pos1 = data.pose.pose.position
        self.pos1 = pos1

    def callback_2(self, data: Odometry):
        pos2 = data.pose.pose.position
        self.pos2 = pos2

    def callback_3(self, data: Odometry):
        pos3 = data.pose.pose.position
        self.pos3 = pos3

    def callback_4(self, data: Odometry):
        pos4 = data.pose.pose.position
        self.pos4 = pos4

    def getUp(self):
        move_msg = Twist()

        for i in range(1, self.number_of_drones + 1):
            move_msg.linear.z = 0.05 * i
            self.publish_once_in_cmd_vel(i, move_msg)

        time.sleep(1)

        for i in range(1, self.number_of_drones + 1):
            move_msg.linear.z = 0.0
            self.publish_once_in_cmd_vel(i, move_msg)

        time.sleep(1)

    def stop(self):
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.linear.y = 0.0
        move_msg.linear.z = 0.0
        for i in range(1, self.number_of_drones + 1):
            self.publish_once_in_cmd_vel(i, move_msg)

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
        pos1 = self.pos1
        pos2 = self.pos2
        pos3 = self.pos3
        pos4 = self.pos4
        posRef = self.posRef

        pos_matrix: List[Point] = [pos1, pos2, pos3, pos4, posRef]

        d1Vis = self.seeOtherDrones(pos1, [pos2, pos3, pos4, posRef])
        d2Vis = self.seeOtherDrones(pos2, [pos1, pos3, pos4, posRef])
        d3Vis = self.seeOtherDrones(pos3, [pos1, pos2, pos4, posRef])
        d4Vis = self.seeOtherDrones(pos4, [pos1, pos2, pos3, posRef])

        comunicate_matrix: List[List[int]] = [[0, d1Vis[0], d1Vis[1], d1Vis[2], d1Vis[3]],
                                            [d2Vis[0], 0, d2Vis[1], d2Vis[2], d2Vis[3]],
                                            [d3Vis[0], d3Vis[1], 0, d3Vis[2], d3Vis[3]],
                                            [d4Vis[0], d4Vis[1], d4Vis[2], 0, d4Vis[3]],
                                            [0, 0, 0, 0, 0]]

        if self.missionCanBeCompleted(comunicate_matrix):
            pass
        else:
            print("Mission can not be completed!")
            self.finish = True
            return
        
        next_positions: List[Point] = self.control_law(pos_matrix, comunicate_matrix)
        
        move_msg = Twist()

        finishArray = [0] * len(pos_matrix)

        for i in range(0, len(pos_matrix)):
            law_x = next_positions[i].x
            law_y = next_positions[i].y

            if abs(law_x) < self.distance_threshold:
                move_msg.linear.x = 0.0
            else:
                move_msg.linear.x = np.clip(law_x, -0.03, 0.03)

            if abs(law_y) < self.distance_threshold:
                move_msg.linear.y = 0.0
            else:
                move_msg.linear.y = np.clip(law_y, -0.03, 0.03)

            self.publish_once_in_cmd_vel(i+1, move_msg)

            if move_msg.linear.x == 0.0 and move_msg.linear.y == 0.0:
                finishArray[i] = 1

        if sum(finishArray) == len(pos_matrix):
            self.finish = True


if __name__ == '__main__':
    control_algorithm()
