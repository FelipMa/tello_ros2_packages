#!/usr/bin/env python3

from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import numpy as np
import threading
from typing import List
import copy
import matplotlib.pyplot as plt
import random
import networkx as nx
from tello_controller import tello_controller

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
    
    def control_law(self, information_state_list: List[Point], comunicate_matrix: List[List[int]]) -> List[Point]:
        # information state is the coordination variable
        # in our case, it is the position drones agree to go to
        next_positions = copy.deepcopy(information_state_list)

        for i in range(0, len(information_state_list)):
            ctrl_law_Ux = 0.0
            ctrl_law_Uy = 0.0
            comunications = 1

            for j in range(0, len(information_state_list)):
                if comunicate_matrix[i][j]: # if drone i can see drone j
                    comunications += 1
                    
                    temp_Ux = comunicate_matrix[i][j] * (information_state_list[i].x - information_state_list[j].x)
                    ctrl_law_Ux -= temp_Ux

                    temp_Uy = comunicate_matrix[i][j] * (information_state_list[i].y - information_state_list[j].y)
                    ctrl_law_Uy -= temp_Uy

            # ctrl_law_U is the information control input, derivative of the drone i desired position
            # ctrl_law_U / comunications is how much distance drone i should move in the direction, based on our own rule
            # when consensus is found, ctrl_law_U tends to be 0, so the drones next positions tend to be the same
            next_positions[i].x += ctrl_law_Ux / comunications
            next_positions[i].y += ctrl_law_Uy / comunications

        return next_positions
    
    def consensus_algorithm(self):

        # drone states are their desired positions
        # initially, the states are their current positions
        consensus_agreed_pos_list: List[Point]  = list(map(lambda drone: drone.get_consensus_agreed_state() if drone.get_consensus_agreed_state() else drone.get_pos(), self.drones))
    
        pos_list: List[Point] = [drone.get_pos() for drone in self.drones]
        self.trajectory_plot.append(pos_list)

        # Check if the drones are close enough to each other
        if max([pos.x for pos in pos_list]) - min([pos.x for pos in pos_list]) <= self.distance_threshold and max([pos.y for pos in pos_list]) - min([pos.y for pos in pos_list]) <= self.distance_threshold:
            self.finish = True
            return

        #if self.comunicate_matrix is None:
        #self.comunicate_matrix = self.random_connected_graph(self.number_of_drones)

        self.comunicate_matrix = self.graph_based_on_vision(pos_list)

        comunicate_matrix: List[List[int]] = self.comunicate_matrix
        
        next_positions: List[Point] = self.control_law(consensus_agreed_pos_list, comunicate_matrix)

        # Check if consensus is found
        if max([pos.x for pos in consensus_agreed_pos_list]) - min([pos.x for pos in consensus_agreed_pos_list]) > self.consensus_agreement_threshold and max([pos.y for pos in consensus_agreed_pos_list]) - min([pos.y for pos in consensus_agreed_pos_list]) > self.consensus_agreement_threshold:
            self.next_positions_plot.append(next_positions)

        for i in range(0, self.number_of_drones):
            self.drones[i].move_to_pos(next_positions[i])
            self.drones[i].set_consensus_agreed_state(next_positions[i])

        time.sleep(0.1)

        return

    def generate_plot(self) -> None:
        trajectory_2d_fig = plt.figure(figsize=(6, 6))
        trajectory_ax = trajectory_2d_fig.add_subplot()
        trajectory_ax.set_xlabel('x')
        trajectory_ax.set_ylabel('y')
        plt.suptitle("Consensus algorithm trajectory")
        trajectory_ax.set_title(f'Consensus point: X = {self.trajectory_plot[-1][0].x}, Y = {self.trajectory_plot[-1][0].y}', fontsize=10)
        trajectory_ax.set_aspect('auto', adjustable='box')

        for i in range(0, self.number_of_drones):
            x = [pos.x for pos in [pos_list[i] for pos_list in self.trajectory_plot]]
            y = [pos.y for pos in [pos_list[i] for pos_list in self.trajectory_plot]]
            trajectory_ax.plot(x, y, label=f'Drone {i+1}')
        
        trajectory_ax.legend(loc='upper right')

        consensus_xy_fig, axs = plt.subplots(2, 1, figsize=(10, 9))

        consensus_x_ax: plt.Axes = axs[0]
        consensus_y_ax: plt.Axes = axs[1]

        consensus_x_ax.set_xlabel('iterations')
        consensus_x_ax.set_ylabel('x')
        consensus_x_ax.set_title('Consensus algorithm x coordinate')
        consensus_x_ax.set_aspect('auto', adjustable='box')

        for i in range(0, self.number_of_drones):
            y = [next_pos.x for next_pos in [pos_list[i] for pos_list in self.next_positions_plot]]
            x = [i for i in range(0, len(y))]
            consensus_x_ax.step(x, y, label=f'Drone {i+1}', where='post')

        consensus_x_ax.legend(loc='upper right')

        consensus_y_ax.set_xlabel('iterations')
        consensus_y_ax.set_ylabel('y')
        consensus_y_ax.set_title('Consensus algorithm y coordinate')
        consensus_y_ax.set_aspect('auto', adjustable='box')

        for i in range(0, self.number_of_drones):
            y = [next_pos.y for next_pos in [pos_list[i] for pos_list in self.next_positions_plot]]
            x = [i for i in range(0, len(y))]
            consensus_y_ax.step(x, y, label=f'Drone {i+1}', where='post')

        consensus_y_ax.legend(loc='upper right')

        plt.subplots_adjust(hspace=0.5)

        local_time = time.localtime()
        current_date = f"{local_time.tm_year}-{local_time.tm_mon}-{local_time.tm_mday}_{local_time.tm_hour}-{local_time.tm_min}-{local_time.tm_sec}"

        graph_fig = plt.figure(figsize=(6, 6))
        graph_ax = graph_fig.add_subplot()

        graph_ax.set_title('Communication graph')

        graph = nx.from_numpy_array(np.array(self.comunicate_matrix))

        label_dict = {}
        for i in range(0, self.number_of_drones):
            label_dict[i] = f'D{i+1}'

        nx.draw_circular(graph, labels=label_dict, with_labels=True, ax=graph_ax)

        path = 'pictures/rendezvous'
        self.create_folder(path)
        trajectory_2d_fig.savefig(f'{path}/trajectory/{current_date}.png')
        consensus_xy_fig.savefig(f'{path}/consensus/{current_date}.png')
        graph_fig.savefig(f'{path}/graph/{current_date}.png')

        plt.show()

    def create_folder(self, path: str) -> None:
        Path(path).mkdir(parents=True, exist_ok=True)
        Path(path + '/trajectory').mkdir(parents=True, exist_ok=True)
        Path(path + '/consensus').mkdir(parents=True, exist_ok=True)
        Path(path + '/graph').mkdir(parents=True, exist_ok=True)

if __name__ == '__main__':
    control_algorithm()
