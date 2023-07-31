import math
import numpy as np
from numpy import random
from random import randrange
import networkx as nx
from scipy import spatial
from PIL import Image
import matplotlib.pyplot as plt

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost

# Class for Informed RRT*
class Informed_RRTSTAR:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        self.goalcost = 0.0                   # cost from start to goal

    def init_map(self):
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    def euclidean_distance(self, node1, node2):
        euclidean_distance = math.sqrt((node1.row - node2.row) ** 2 + (node1.col - node2.col) ** 2)
        return euclidean_distance

    def check_collision(self, node1, node2):
        # Check if there is any obstacle between two nodes
        points_between = zip(np.linspace(node1.row, node2.row, dtype=int), 
                             np.linspace(node1.col, node2.col, dtype=int))
        for point in points_between:
            if self.map_array[point[0]][point[1]] == 0:
                return True
        return False

    def get_new_sample_point(self, goal_bias):
        # Randomly sample a new point; with probability "goal_bias", sample the goal point
        a = randrange(100)
        if a <= goal_bias:
            y = True
        else:
            y = False
        if y == False:
            p1rows = random.randint(self.size_row)
            p1cols = random.randint(self.size_col)
            new_point = Node(p1rows, p1cols)
            return new_point
        else:
            return self.goal

    def get_nearest_node(self, point):
        # Find the nearest node to the given point
        euclidean_distance_to_new_node = []
        minindex = 0
        for i in range(len(self.vertices)):
            euclidean_distance = self.euclidean_distance(self.vertices[i], point)
            euclidean_distance_to_new_node.append(euclidean_distance)

        minindex = euclidean_distance_to_new_node.index(min(euclidean_distance_to_new_node))
        return self.vertices[minindex]
    
    def extend_towards_point(self, p1, p2):
        step = 10 
        euclidean_distance_1 = self.euclidean_distance(p1, p2)
        if euclidean_distance_1 > step:
            euclidean_distance_1 = step
        
        theta = math.atan2(p2.col - p1.col, p2.row - p1.row)
        new_node1 = Node((int((p1.row + euclidean_distance_1 * math.cos(theta)))),
                         (int((p1.col + euclidean_distance_1 * math.sin(theta)))))

        return new_node1

    def find_neighbors(self, new_node, neighbor_size):
        # Find neighbors within neighbor_size distance from the new_node
        neighbors = []
        free_neighbors = []
        for i in range(len(self.vertices)):
            euclidean_distance_rrt = self.euclidean_distance(new_node, self.vertices[i])
            if euclidean_distance_rrt <= neighbor_size:
                neighbors.append(self.vertices[i])

        for i in range(len(neighbors)):
            # Find only the neighboring nodes that do not have obstacles in between
            collision_1 = self.check_collision(neighbors[i], new_node)
            if collision_1 == True:
                continue
            else:
                free_neighbors.append(neighbors[i])
        free_neighbors.remove(new_node)
        return free_neighbors

    def rewire(self, new_node, neighbors):
        if neighbors == []:
            # If there are no neighbors, no need to rewire
            return

        # Compute the Euclidean distance from the new node to the neighbor nodes
        euclidean_distances = [self.euclidean_distance(new_node, node) for node in neighbors]

        # Rewire the new node
        # Compute the least potential cost
        costs = [euclidean_distance + self.path_cost(self.start, neighbors[i]) for i, euclidean_distance in enumerate(euclidean_distances)]
        indices = np.argsort(np.array(costs))

        # Check collision and connect the best node to the new node
        for i in indices:
            if not self.check_collision(new_node, neighbors[i]):
                new_node.parent = neighbors[i]
                new_node.cost = euclidean_distances[i]
                break

        # Rewire new_node's neighbors
        for i, node in enumerate(neighbors):
            # Compute new cost
            new_cost = self.path_cost(self.start, new_node) + euclidean_distances[i]
            # If new cost is lower and there are no obstacles in between
            if self.path_cost(self.start, node) > new_cost and \
               not self.check_collision(node, new_node):
                node.parent = new_node
                node.cost = euclidean_distances[i]

    def path_cost(self, start_node, end_node):
        # Compute path cost starting from start node to end node
        cost = 0
        curr_node = end_node
        while start_node.row != curr_node.row or start_node.col != curr_node.col:
            # Keep tracing back until finding the start_node 
            # or no path exists
            parent = curr_node.parent
            if parent is None:
                print("Invalid Path")
                return 0
            cost += curr_node.cost
            curr_node = parent
        
        return cost

    def sample_unit_circle(self):
        # Sample a point within the unit circle
        while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                a = np.array([[x], [y]])
            else:
                continue
            return a

    def ellipse(self, cmax, start, goal, center, cmin):
        while True:
            extend_towards_point = math.atan2(goal.col - start.col, goal.row - start.row)
            c, s = np.cos(extend_towards_point), np.sin(extend_towards_point)
            C1 = np.array(((c, -s), (s, c)))          # Rotation matrix of the rotated ellipse calculated using theta above
            r1 = cmax / 2                             # Major axis
            r2 = (math.sqrt(cmax ** 2 - cmin ** 2)) / 2    # Minor axis
            L = np.diag([r1, r2])                     # Diagonal matrix to convert ellipse from circle
            x_ball = self.sample_unit_circle()

            xr = np.dot(np.dot(C1, L), x_ball) + center   # Final transformation from circle to rotated and translated ellipse
            xr1 = xr[0][0]
            xr2 = xr[1][0]
            xrand = Node(int(xr1), int(xr2))
            if xr1 < self.size_row and xr2 < self.size_col:
                return xrand
            else:
                continue

    def visualize_map(self, title):
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='blue')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='blue')

        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col and cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='red')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='red')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='green')
        plt.text(self.start.col, self.start.row, 'Start', color='black', fontsize=14, va='top', ha='center')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='green')
        plt.text(self.goal.col, self.goal.row, 'End', color='black', fontsize=14, va='top', ha='center')

        # Show image
        plt.show()

    def Informed_RRT_star(self, n_pts=1000, neighbor_size=20):
        self.init_map()
        goal_bias = 4
        step = 10
        cmin = self.euclidean_distance(self.start, self.goal)
        xcenter = np.array([[(self.start.row + self.goal.row) / 2], [(self.start.col + self.goal.col) / 2]])
        nearnode = []
        for i in range(n_pts):
            if self.found == True:
                # If the goal is found, sample within the ellipse
                new_p = self.ellipse(self.goalcost, self.start, self.goal, xcenter, cmin)
            else:
                # Else, sample randomly on the map
                new_p = self.get_new_sample_point(goal_bias)

            if self.map_array[new_p.row][new_p.col] == 0:
                continue
            else:
                nearestnode = self.get_nearest_node(new_p)
                newnode = self.extend_towards_point(nearestnode, new_p)
                collision = self.check_collision(newnode, nearestnode)
                if collision == True:
                    continue
                else:
                    self.vertices.append(newnode)
                    neighbors = self.find_neighbors(newnode, 20)
                    self.rewire(newnode, neighbors)

                euclidean_distance_to_goal = self.euclidean_distance(newnode, self.goal)
                if euclidean_distance_to_goal <= step:
                    self.goal.parent = newnode
                    self.goal.cost = euclidean_distance_to_goal
                    self.vertices.append(self.goal)
                    self.found = True
                    self.goalcost = self.path_cost(self.start, self.goal)
                    nearnode.append(newnode)

        self.rewire(self.goal, nearnode)    # Rewire again all the nodes close to the goal

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goalcost
            print("It took %d nodes to find the path using Informed RRT*" % steps)
            print("The length of path is %.2f" % length)
        else:
            print("No path found")

        # Draw result
        self.visualize_map(title="Informed RRT*")