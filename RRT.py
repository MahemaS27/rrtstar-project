import matplotlib.pyplot as plt
import matplotlib as matplot
import numpy as np
import math
from numpy import random
from PIL import Image
from random import randrange
import time

# Class for each tree node
class TreeNode:
    def __init__(self, row, col):
        self.row = row        # Coordinate (row)
        self.col = col        # Coordinate (column)
        self.parent = None    # Parent node
        self.cost = 0.0       # Cost

# Class for RRT Planner
class RRTPlanner:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # Map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # Map size (rows)
        self.size_col = map_array.shape[1]    # Map size (columns)

        self.start = TreeNode(start[0], start[1]) # Start node
        self.goal = TreeNode(goal[0], goal[1])    # Goal node
        self.vertices = []                    # List of nodes in the RRT tree
        self.found = False                    # Found flag to indicate if the goal is reached

    def init_map(self):
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    def euclidean_distance(self, node1, node2):
        # Calculate Euclidean distance between two nodes
        distance = math.sqrt((node1.row - node2.row) ** 2 + (node1.col - node2.col) ** 2)
        return distance

    def check_collision(self, node1, node2):
        # Check if there is a collision between two nodes by checking the points in between
        points_between = zip(np.linspace(node1.row, node2.row, dtype=int),
                             np.linspace(node1.col, node2.col, dtype=int))
        for point in points_between:
            if self.map_array[point[0]][point[1]] == 0:
                return False
        return True

    def get_new_sample_point(self, goal_bias):
        # Generate a new sample point, with a probability (goal_bias) of picking the goal point
        a = randrange(100)
        if a <= goal_bias:
            return self.goal
        else:
            p1rows = random.randint(self.size_row)
            p1cols = random.randint(self.size_col)
            new_point = TreeNode(p1rows, p1cols)
            return new_point

    def get_nearest_neighbor(self, point):
        # Find the nearest neighbor to a given point among the existing vertices in the RRT tree
        dist_new_node = [self.euclidean_distance(node, point) for node in self.vertices]
        minindex = dist_new_node.index(min(dist_new_node))
        return self.vertices[minindex]

    def find_neighbors(self, new_node, neighbor_size):
        # Find neighbors of a new node within a certain distance (neighbor_size)
        neighbors = [node for node in self.vertices if self.euclidean_distance(new_node, node) <= neighbor_size]
        freeneighbors = [neighbor for neighbor in neighbors if self.check_collision(neighbor, new_node)]
        return freeneighbors

    def rewire_neighbors(self, new_node, neighbors):
        # Rewire neighbors of a new node to improve the path
        checkdis = []
        free = []
        for neighbor in neighbors:
            if not self.check_collision(neighbor, new_node):
                continue
            free.append(neighbor)

        for neighbor in free:
            cost1 = neighbor.cost + self.euclidean_distance(neighbor, new_node)
            checkdis.append(cost1)

        minindex = checkdis.index(min(checkdis))
        new_node.parent = neighbors[minindex]
        new_node.cost = neighbors[minindex].cost + int(self.euclidean_distance(neighbors[minindex], new_node))

        for neighbor in free:
            initcost = neighbor.cost
            rewiredcost = int(self.euclidean_distance(new_node, neighbor)) + new_node.cost

            if initcost > rewiredcost:
                neighbor.parent = new_node
                neighbor.cost = rewiredcost

    def extend_towards_point(self, p1, p2):
        # Extend a node towards a given point within a certain step size
        step = 10
        distance_1 = self.euclidean_distance(p1, p2)
        if distance_1 > step:
            distance_1 = step

        theta = math.atan2(p2.col - p1.col, p2.row - p1.row)
        new_node = TreeNode(int((p1.row + distance_1 * math.cos(theta))), int((p1.col + distance_1 * math.sin(theta))))
        return new_node

    def visualize_map(self, title):
            # Create a plot to visualize the map, RRT tree, and path
            fig, ax = plt.subplots(1)
            img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
            ax.imshow(img)

            # Draw RRT tree nodes and edges
            for node in self.vertices[1:-1]:
                plt.plot(node.col, node.row, markersize=3, marker='o', color='blue')  # Node color as blue
                plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='blue')  # Edge color as blue

            # Draw the final path if found
            if self.found:
                cur = self.goal
                while cur.col != self.start.col and cur.row != self.start.row:
                    plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='red')  # Path color as red
                    cur = cur.parent
                    plt.plot(cur.col, cur.row, markersize=3, marker='o', color='red')  # Node color as red

            # Draw start and goal nodes
            plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='green')  # Start node color as green
            plt.text(self.start.col, self.start.row, 'Start', color='black', fontsize=14, va='top', ha='center')  # Start node label in black below the node
            plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='green')  # Goal node color as green
            plt.text(self.goal.col, self.goal.row, 'End', color='black', fontsize=14, va='top', ha='center')  # Goal node label in black below the node

            # Add title to the plot
            plt.title(title)
            
            # Show the plot
            plt.show()




    def RRT(self, n_pts=1000):
        # RRT algorithm implementation
        self.init_map()
        goal_bias = 4
        step = 10
        for i in range(n_pts):
            new_sample = self.get_new_sample_point(goal_bias)
            if self.map_array[new_sample.row][new_sample.col] == 0:
                continue
            else:
                nearest_node = self.get_nearest_neighbor(new_sample)
                new_node = self.extend_towards_point(nearest_node, new_sample)
                collision = self.check_collision(nearest_node, new_node)
                if collision:
                    self.vertices.append(new_node)
                    new_node.cost = int(nearest_node.cost + self.euclidean_distance(nearest_node, new_node))
                    new_node.parent = nearest_node
                    dist = self.euclidean_distance(new_node, self.goal)
                    if dist <= step:
                        x = new_node
                        nearest_node = x
                        new_node = self.goal
                        new_node.parent = nearest_node
                        new_node.cost = int(nearest_node.cost + self.euclidean_distance(nearest_node, new_node))
                        self.vertices.append(new_node)
                        self.found = True
                        break

        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the path using RRT" % steps)
            print("The length of path is %.2f" % length)
        else:
            
            print("No path found")

        self.visualize_map(title = "RRT")

    def RRT_star(self, n_pts, neighbor_size, name):
        # RRT* algorithm implementation
        self.init_map()
        print('init map')
        goal1 = False
        goal_bias = 4
        step = 10
        start_time = time.time()
        for i in range(n_pts):
            new_sample = self.get_new_sample_point(goal_bias)
            if self.map_array[new_sample.row][new_sample.col] == 0:
                continue
            else:
                nearest_node = self.get_nearest_neighbor(new_sample)
                if goal1:
                    new_node = self.goal
                else:
                    new_node = self.extend_towards_point(nearest_node, new_sample)
                    collision = self.check_collision(new_node, nearest_node)
                if collision:
                    neighbors = self.find_neighbors(new_node, neighbor_size)
                    self.rewire_neighbors(new_node, neighbors)
                    dist = self.euclidean_distance(new_node, self.goal)
                    if dist <= step:
                        goal1 = True
                    self.vertices.append(new_node)

                    if dist == 0:
                        self.found = True
                        goal1 = False
        print('finishloop')
        if self.found:
            print(name)
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the path using RRT*" % steps)
            print("The length of path is %.2f" % length)
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"Elapsed time: {elapsed_time} seconds")
            #self.visualize_map("RRT*")
        else:
            print(name)
            print("No path found")
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"Elapsed time: {elapsed_time} seconds")
            #self.visualize_map("RRT*")