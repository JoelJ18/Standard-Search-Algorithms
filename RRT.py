# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import math
import networkx as nx
from scipy import spatial

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
        d=math.sqrt((node1.row-node2.row)**2 + (node1.col-node2.col)**2)  #same as the function in the PRM file,except this takes nodes as inputs
        return d
    
    def check_collision(self, node1, node2):                              #same as the function in the PRM file,except this takes nodes as inputs
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        ax=np.linspace(node1.row,node2.row,dtype=int)
        ay=np.linspace(node1.col,node2.col,dtype=int)
        for i in range(0,len(ax)):
            tempax=ax[i]
            tempay=ay[i]
            if(self.map_array[tempax][tempay]==0):
                return True
        return False


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###

        testgoal=np.round(np.random.uniform(1,100))                      #generates a random number from 1-100
        if(testgoal<=goal_bias):                                         #occasionally returns the goal node as the new point, rate is controlled by goal_bias
            return self.goal
        else:
            rand_x=int(np.round(np.random.uniform(0,self.size_row-1)))   
            rand_y=int(np.round(np.random.uniform(0,self.size_col-1)))
            return Node(rand_x,rand_y)                                   #otherwise returns a new node with random coordinates


    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        dlist=[]
        for i in self.vertices:
            dlist.append(self.dis(i,point))                             #generates a list of the distances of all the nodes to the point and returns the minimum
        return self.vertices[dlist.index(min(dlist))]


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbors=[]
        for i in self.vertices:
            if(self.dis(new_node,i)<=neighbor_size and self.check_collision(new_node,i)==False):   #if the distance to the new node from any node in the node list is less than or equal to the neighbor_size
                neighbors.append(i)                                                                #and has no obstacle in the line connecting them, the node in the node list is appended to the neighbors list

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
        clist=[]                                                
                                                                    #collision checks are not needed as they are already made in other functions
        for i in neighbors:
            clist.append(i.cost+self.dis(new_node,i))               #finds the costs to get to the new node from all the neighbors and stores it in clist 

        new_node.parent=neighbors[clist.index(min(clist))]          #sets the parent of the new node to the neighbor through which minimum cost is achieved
        new_node.cost=min(clist)                                    #sets the new cost to the new node

        for i in neighbors:                                         #rewiring of all the neighbors starts
            rcost=new_node.cost+self.dis(new_node,i)                #stores the rewired cost, if a neighbors parent was the new node instead of what it is currently
            if(i.cost>rcost):                                       #if the neighbors current cost is higher than what it is if its connected to the new node, it sets the new node as the parent and gets the rewired cost
                i.parent=new_node
                i.cost=rcost

    def extend(self, snode, rnode, step):                                                     #function to generate a node on the line connecting snode and rnode having a distance of step from snode
        
        d=self.dis(snode,rnode)
        if(d>step):
            d=step

        theta=math.atan2(rnode.col-snode.col,rnode.row-snode.row)
        return Node(int(snode.row+(d*math.cos(theta))),int(snode.col+(d*math.sin(theta))))


    
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
        goal_bias=5                                                                        #goal bias set at 3% so newp will be the goal node 5% of the time
        step=10                                                                            #extension distance from a node
        for i in range(0,n_pts):
            newp=self.get_new_point(goal_bias)                                             #A new point is generated (as a node) (newp)
            if(self.map_array[newp.row][newp.col]!=0):                                     #if not an obstacle then
                nearn=self.get_nearest_node(newp)                                          #A node nearest to the random point is found from the node list (nearn)
                extn=self.extend(nearn,newp,step)                                          #A node is generated on the line connecting nearn and newp with a distance of step from nearm
                if(self.check_collision(nearn,extn)==False):                               #collision check to see if the line connecting extended node (extn) to nearn has an obstacle, if not then
                    extn.cost=int(nearn.cost+self.dis(nearn,extn))                         #cost and parent node is assigned to extn and it is appended to the node list
                    extn.parent=nearn
                    self.vertices.append(extn)
                    if(extn.row==self.goal.row and extn.col==self.goal.col):               #if the extn has the same coords as the goal node then we set found to true to indicate we reached the goal
                        self.found=True
                        self.goal=extn                                                     #we set the goal node as extn to carry over the cost and parent information
                        break                                                              #we break as we dont need to keep iterating as we found the goal

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
        goal_bias=10                                                                        #Similar to RRT ,goal bias is set to 8% here it can be changed
        step=10                         
        for i in range(0,n_pts):
            newp=self.get_new_point(goal_bias)
            if(self.map_array[newp.row][newp.col]!=0):
                nearn=self.get_nearest_node(newp)
                extn=self.extend(nearn,newp,step)
                if(self.check_collision(nearn,extn)==False):
                    n=self.get_neighbors(extn,neighbor_size)                                #Instead of directly assigning costs and parent to extn node as in RRT, we find the neighbors around extn using neighbor_size as a radius
                    self.rewire(extn,n)                                                     #We rewire extn and the neighbors, it is here the costs and parent nodes are assigned.
                    self.vertices.append(extn)
                    if(extn.row==self.goal.row and extn.col==self.goal.col):                #Again similar to RRT but we do not break when we find the goal, as on further iterations the path will get more optimized through the rewiring
                        self.found=True
                        self.goal=extn                                                 

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
