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
import matplotlib.pyplot as plt
import random

class control_algorithm():

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('control_algorithm')
        self.drones: List[tello_controller] = []
        self.number_of_drones = 5
        self.finish = False
        self.distance_threshold = 0.05
        self.consensus_agreement_threshold = 0.001
        self.trajectory_plot: List[List[Point]] = []
        self.next_positions_plot: List[List[Point]] = []
        self.comunicate_matrix = None

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

        # An arbitrary drone can start at the target position and stay there, in order to have a reference for the consensus algorithm
        # self.drones[0].at_target = True

        # Start the consensus algorithm
        while rclpy.ok():
            self.consensus_algorithm()
            if self.finish:
                self.stop()
                print("Finished!")
                break

        # Generate plot
        self.generate_plot()

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
            self.drones[i].at_target = True

    def seeOtherDrones(self, dronePos: Point, otherPos: List[Point]) -> List[int]:
        radius = 5.1
        seeArray = [0] * len(otherPos)

        for i in range(0, len(otherPos)):
            if (dronePos.x - otherPos[i].x)**2 + (dronePos.y - otherPos[i].y)**2 <= radius**2:
                seeArray[i] = 1
            else:
                seeArray[i] = 0

        return seeArray
    
    def graph_based_on_vision(self, pos_list: List[Point]) -> List[List[int]]:
        visions: List[List[int]] = []

        for i in range(0, self.number_of_drones):
            visions.append(self.seeOtherDrones(pos_list[i], [pos_list[j] for j in range(0, len(pos_list)) if j != i]))

        graph: List[List[int]] = []
        for i in range(0, self.number_of_drones):
            graph.append(visions[i])
            graph[i].insert(i, 0)

        ref_vis = [0] * (self.number_of_drones+1)
        graph.append(ref_vis)
        
        return graph
    
    def random_connected_graph(self, n) -> List[List[int]]:
        # Initialize an empty matrix
        graph_matrix = [[0 for _ in range(n)] for _ in range(n)]

        while not self.graph_is_connected(graph_matrix):
            # Randomly choose two nodes,
            # Randomness implies that the graph is not necessarily a spanning tree
            node1 = random.randint(0, n-1)
            node2 = random.randint(0, n-1)
            if node1 != node2 and graph_matrix[node1][node2] == 0:
                graph_matrix[node1][node2] = 1
                graph_matrix[node2][node1] = 1

        return graph_matrix
    
    def graph_is_connected(self, graph):
        n = len(graph)  # Number of vertices
        visited = [False] * n

        def dfs(node): # Depth-first search
            visited[node] = True
            for neighbor in range(n):
                if graph[node][neighbor] == 1 and not visited[neighbor]:
                    dfs(neighbor)

        # Start DFS from any unvisited node
        for node in range(n):
            if not visited[node]:
                dfs(node)
                break

        # Check if all nodes were visited
        return all(visited)
    
    def control_law(self, positions_list: List[Point], comunicate_matrix: List[List[int]]) -> List[Point]:
        next_positions = copy.deepcopy(positions_list)

        for i in range(0, len(positions_list)):
            ctrl_law_Ux = 0.0
            ctrl_law_Uy = 0.0
            comunications = 1

            for j in range(0, len(positions_list)):
                if comunicate_matrix[i][j]: # if drone i can see drone j
                    comunications += 1
                    
                    temp_Ux = comunicate_matrix[i][j] * (positions_list[i].x - positions_list[j].x)
                    ctrl_law_Ux = ctrl_law_Ux - temp_Ux

                    temp_Uy = comunicate_matrix[i][j] * (positions_list[i].y - positions_list[j].y)
                    ctrl_law_Uy = ctrl_law_Uy - temp_Uy

            # ctrl_law_Ux / comunications is how much drone i should move in the x direction
            # ctrl_law_Uy / comunications is how much drone i should move in the y direction
            # when consensus is found, ctrl_law_Ux / comunications and ctrl_law_Uy / comunications tend to 0,
            # so the drones next positions tend to be the same
            next_positions[i].x += ctrl_law_Ux / comunications
            next_positions[i].y += ctrl_law_Uy / comunications

        return next_positions
    
    def consensus_algorithm(self):

        consensus_agreed_pos_list: List[Point] = [drone.get_consensus_agreed_pos() for drone in self.drones] if all([drone.get_consensus_agreed_pos() for drone in self.drones]) else [drone.get_pos() for drone in self.drones]
    
        pos_list: List[Point] = [drone.get_pos() for drone in self.drones]
        self.trajectory_plot.append(pos_list)

        # Check if the drones are close enough to each other
        if max([pos.x for pos in pos_list]) - min([pos.x for pos in pos_list]) <= self.distance_threshold and max([pos.y for pos in pos_list]) - min([pos.y for pos in pos_list]) <= self.distance_threshold:
            self.finish = True
            return

        if self.comunicate_matrix is None:
            self.comunicate_matrix = self.random_connected_graph(self.number_of_drones)

        comunicate_matrix: List[List[int]] = self.comunicate_matrix
        
        next_positions: List[Point] = self.control_law(consensus_agreed_pos_list, comunicate_matrix)

        # Check if consensus is found
        if max([pos.x for pos in consensus_agreed_pos_list]) - min([pos.x for pos in consensus_agreed_pos_list]) > self.consensus_agreement_threshold and max([pos.y for pos in consensus_agreed_pos_list]) - min([pos.y for pos in consensus_agreed_pos_list]) > self.consensus_agreement_threshold:
            self.next_positions_plot.append(next_positions)

        for i in range(0, self.number_of_drones):
            self.drones[i].move_to_pos(next_positions[i])

    def generate_plot(self) -> None:
        trajectory_2d = plt.figure(figsize=(8, 8))
        trajectory_ax = trajectory_2d.add_subplot()
        trajectory_ax.set_xlabel('x')
        trajectory_ax.set_ylabel('y')
        trajectory_ax.set_title('Consensus algorithm trajectory')
        trajectory_ax.set_aspect('auto', adjustable='box')

        for i in range(0, self.number_of_drones):
            x = [pos.x for pos in [pos_list[i] for pos_list in self.trajectory_plot]]
            y = [pos.y for pos in [pos_list[i] for pos_list in self.trajectory_plot]]
            trajectory_ax.plot(x, y, label=f'Drone {i+1}')
        
        trajectory_ax.legend(loc='upper right')

        consensus_xy, axs = plt.subplots(2, 1, figsize=(8, 8))

        consensus_x_ax: plt.Axes = axs[0]
        consensus_y_ax: plt.Axes = axs[1]

        consensus_x_ax.set_xlabel('iterations')
        consensus_x_ax.set_ylabel('x')
        consensus_x_ax.set_title('Consensus algorithm x coordinate')
        consensus_x_ax.set_aspect('auto', adjustable='box')

        for i in range(0, self.number_of_drones):
            x = [next_pos.x for next_pos in [pos_list[i] for pos_list in self.next_positions_plot]]
            consensus_x_ax.plot(x, label=f'Drone {i+1}')

        consensus_x_ax.legend(loc='upper right')

        consensus_y_ax.set_xlabel('iterations')
        consensus_y_ax.set_ylabel('y')
        consensus_y_ax.set_title('Consensus algorithm y coordinate')
        consensus_y_ax.set_aspect('auto', adjustable='box')

        for i in range(0, self.number_of_drones):
            y = [next_pos.y for next_pos in [pos_list[i] for pos_list in self.next_positions_plot]]
            consensus_y_ax.plot(y, label=f'Drone {i+1}')

        consensus_y_ax.legend(loc='upper right')

        plt.subplots_adjust(hspace=0.5)

        print(x[-1])
        print(y[-1])

        local_time = time.localtime()
        current_date = f"{local_time.tm_mday}-{local_time.tm_mon}-{local_time.tm_year}_{local_time.tm_hour}-{local_time.tm_min}-{local_time.tm_sec}"

        trajectory_2d.savefig(f'pictures/trajectory/trajectory-{current_date}.png')
        consensus_xy.savefig(f'pictures/consensus/consensus-{current_date}.png')

        plt.show()

class tello_controller():

    def __init__(self, node: Node, ns: str):
        self.node = node

        self.pub_takeoff = node.create_publisher(Empty, ns + '/takeoff', 1)
        self.pub_land = node.create_publisher(Empty, ns + '/land', 1)
        self.pub_cmd_vel = node.create_publisher(Twist, ns + '/cmd_vel', 1)
        
        self.sub_pos = node.create_subscription(Odometry, ns + '/odom', self.odom_callback, 1)

        self.max_velocity = 0.03

        self.pos = None
        self.consensus_agreed_pos = None

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
    
    def get_consensus_agreed_pos(self) -> Point:
        return self.consensus_agreed_pos
    
    def move_to_pos(self, target_pos: Point) -> None:
        self.consensus_agreed_pos = target_pos
        msg = Twist()

        if self.at_target:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
        else:
            msg.linear.x = np.clip(target_pos.x - self.pos.x, -self.max_velocity, self.max_velocity)
            msg.linear.y = np.clip(target_pos.y - self.pos.y, -self.max_velocity, self.max_velocity)

        self.publish_velocity(msg)

if __name__ == '__main__':
    control_algorithm()
