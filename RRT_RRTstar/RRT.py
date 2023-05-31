# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
from random import choices
from random import randint
import math


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        dist = math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)
        return dist

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        if node1[1]!= node2[1]:
            slope = (node1[0] - node2[0]) / (node2[1] - node1[1])
            c = (self.size_row - node1[0]) - (slope * node1[1])
            if node1[1] < node2[1]:
                start = node1[1]
            elif node2[1] < node1[1]:
                start = node2[1]
            for i in range(abs(int(node1[1] - node2[1])) + 1 ):
                col = int(start + i)
                row = int(self.size_row - (slope * col + c))
                if col < self.size_col and row < self.size_row:
                    if self.map_array[row,col] == 0:
                        return False
        else:
            if node1[0] < node2[0]:
                start = node1[0]
            elif node2[0] < node1[0]:
                start = node2[0]
            for i in range(abs(int(node1[0] - node2[0])) + 1):
                col = int(node1[1])
                row = int(start + i)
                if col < self.size_col and row < self.size_row:
                    if self.map_array[row,col] == 0:
                        return False
        return True


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        temp = []
        for i in range(len(self.vertices)):
            temp.append([self.vertices[i].row, self.vertices[i].col])
        pop = ['goal', 'random']
        weights= [goal_bias, 1 - goal_bias]
        res= choices(pop,weights)
        if res == 'goal':
            point = [self.goal.row, self.goal.col]
        else:
            while(True):
                r = randint(0, self.size_row -1)
                c = randint(0, self.size_col -1)
                point = [r,c]
                if point not in temp:
                    break
        return point
    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        min_dist = float('inf')
        for i in range(len(self.vertices)):
            currPoint = [self.vertices[i].row, self.vertices[i].col]
            dist = self.dis(point, currPoint)
            if dist < min_dist:
                nearNode = self.vertices[i]
                min_dist = dist
        return nearNode
    
    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbors = []
        for i in range(len(self.vertices)):
            currPoint = [self.vertices[i].row, self.vertices[i].col]
            dist = self.dis(new_node, currPoint)
            if dist < neighbor_size:
                neighbors.append(self.vertices[i])    

        return neighbors


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.

        '''
        ### YOUR CODE HERE ###
        nodePoint = [new_node.row, new_node.col]
        neighborsNew = []
    
        for i in range(len(neighbors)):
            currPoint = [neighbors[i].row, neighbors[i].col]
            if self.check_collision(nodePoint, currPoint) == True:
                neighborsNew.append(neighbors[i])
            
        minDist = float('inf')
        for i in range(len(neighborsNew)):
            currPoint = [neighborsNew[i].row, neighborsNew[i].col]
            dist = self.dis(nodePoint, currPoint) + neighborsNew[i].cost
            if self.check_collision(nodePoint, currPoint) == True and dist < minDist:
                new_node.parent = neighborsNew[i]
                new_node.cost = dist
                minDist = dist
        
        self.vertices.append(new_node)
        
        for i in range(len(neighborsNew)):
             currPoint = [neighborsNew[i].row, neighborsNew[i].col]
             dist = self.dis(currPoint, nodePoint) + new_node.cost
             if dist < neighborsNew[i].cost:
                 tempNode = Node(neighborsNew[i].row, neighborsNew[i].col)
                 self.vertices.remove(neighborsNew[i])
                 tempNode.parent = new_node
                 tempNode.cost = dist
                 self.vertices.append(tempNode)
                



    def extend(self,point, nearNode, distThresh):
        temp = []
        for i in range(len(self.vertices)):
                temp.append([self.vertices[i].row, self.vertices[i].col])
        nodePoint = [nearNode.row, nearNode.col] 
        extPoint = [temp[0][0], temp[0][1]]
        while(extPoint in temp):
            if point[1]!= nodePoint[1]:
                slope = (point[0] - nodePoint[0]) / (nodePoint[1] - point[1])
                c = (self.size_row - point[0]) - (slope * point[1])
                k = math.sqrt(distThresh**2 / (1 + slope**2))
                col = nodePoint[1] + k
                row = int(self.size_row - (slope * col + c))
                col = int(col)
                if row >= self.size_row:
                    row = row - 1
                if col >= self.size_col:
                    col = col -1
                extPoint = [row, col]
            else:
                col = nodePoint[1]
                row = nodePoint[0] + distThresh * ((point[0] - nodePoint[0]) / abs(point[0] - nodePoint[0]))
                if row >= self.size_row:
                    row = row - 1
                if col >= self.size_col:
                    col = col -1
                extPoint = [row, col]
            distThresh = distThresh - 1
        return extPoint
    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()

    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.
        goal_bias = 0.07
        distThresh = 15
        goalThresh = 10
        for i in range(n_pts):
            point = self.get_new_point(0.07)
            if self.map_array[point[0], point[1]] == 0:
                continue
            nearNode = self.get_nearest_node(point)
            if self.dis(point, [nearNode.row, nearNode.col]) > distThresh:
                point = self.extend(point, nearNode, distThresh)

            if self.check_collision(point, [nearNode.row, nearNode.col]) == True:
                currNode = Node(point[0], point[1])
                currNode.parent = nearNode
                currNode.cost = nearNode.cost + self.dis(point, [nearNode.row, nearNode.col])
                self.vertices.append(currNode)
                if self.dis(point, [self.goal.row, self.goal.col]) < goalThresh:
                    if self.check_collision(point, [self.goal.row, self.goal.col]) == True:
                        self.goal.parent = self.vertices[-1]
                        self.goal.cost = self.vertices[-1].cost + self.dis(point, [self.goal.row, self.goal.col])
                        self.vertices.append(self.goal)
                        self.found = True
                        break

                
        
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.
        goal_bias = 0.07
        distThresh = 10
        goalThresh = 12
        for i in range(n_pts):
            point = self.get_new_point(0.07)
            #print("Before extending point is: ", point)
            if self.map_array[point[0], point[1]] == 0:
                continue
            nearNode = self.get_nearest_node(point)
            if self.dis(point, [nearNode.row, nearNode.col]) > distThresh:
                point = self.extend(point, nearNode, distThresh)
            
            if self.check_collision(point, [nearNode.row, nearNode.col]) == True:
                neighbors = self.get_neighbors(point, neighbor_size)
                newNode = Node(point[0], point[1])
                self.rewire(newNode, neighbors)
                if self.dis(point, [self.goal.row, self.goal.col]) < goalThresh:
                    if self.check_collision(point, [self.goal.row, self.goal.col]) == True:
                        self.goal.parent = self.vertices[-1]
                        self.vertices.append(self.goal)
                        self.goal.cost = newNode.cost + self.dis(point, [self.goal.row, self.goal.col])
                        self.found = True
                        break



        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
