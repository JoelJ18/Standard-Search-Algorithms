# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import math
import networkx as nx
from scipy import spatial


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        ### YOUR CODE HERE ###
        ax=np.linspace(p1[0],p2[0],dtype=int)          #generates a list of points for x and y when combined are the points on the line connecting p1 and p2
        ay=np.linspace(p1[1],p2[1],dtype=int)
        for i in range(0,len(ax)):
            tempax=ax[i]
            tempay=ay[i]
            if(self.map_array[tempax][tempay]==0):     #takes the coordinates for each point on the line and checks if theres an obstacle in the map array
                return True                            #Returns true if there is
        return False




    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        d=math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)  #euclidean distance formula
        return d
        


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        pts=int(math.sqrt(n_pts))                               
        uni_x=np.round(np.linspace(0,self.size_row-1,pts))      #generating a list of equally spaced points for x and y
        uni_y=np.round(np.linspace(0,self.size_col-1,pts))

        for i in uni_x:
            for j in uni_y:
                if(self.map_array[int(i)][int(j)]!=0):          #If the coordinates generated are not obstacles in the map array then we add the coordinate to the samples list
                    self.samples.append([int(i),int(j)])

        
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###

        rand_x=np.round(np.random.uniform(0,self.size_row-1,n_pts))  #generating a list of points randomly from a unifrom distribution for x and y
        rand_y=np.round(np.random.uniform(0,self.size_col-1,n_pts))

        for i in range(0,n_pts):
            tempx=int(rand_x[i])                                     #Picking our x and y from the list
            tempy=int(rand_y[i])
            if(self.map_array[tempx][tempy]!=0):                     #If the coordinates generated are not obstacles in the map array then we add the coordinate to the samples list
                self.samples.append([tempx,tempy])
           
       
    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###

       
        rand_x=np.round(np.random.uniform(0,self.size_row-1,n_pts))  #generating a list of points randomly from a unifrom distribution for x and y
        rand_y=np.round(np.random.uniform(0,self.size_col-1,n_pts))

        for i in range(0,n_pts):
            tempx=int(rand_x[i])                                     #Picking our x and y from the list
            tempy=int(rand_y[i])
            if(self.map_array[tempx][tempy]==0):                     #If the coordinates picked are obstacles in the map array then we have our first point
                flag=0
                while(flag==0):                                      #While loop runs until we find viable coordinates for the second point
                    gaus_x=int(np.round(np.random.normal(tempx,8)))  #generating points randomly from a gaussian/normal distribution for x and y
                    gaus_y=int(np.round(np.random.normal(tempy,8)))
                    if(gaus_x<self.size_row and gaus_y<self.size_col and self.map_array[gaus_x][gaus_y]!=0): #If the coordinates generated are witin limits and not obstacles in the map array then we add the coordinate to the samples list
                        self.samples.append([gaus_x,gaus_y])
                        flag=1                                       #flag changed to show we have found viable coordinates

        
    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###

        rand_x=np.round(np.random.uniform(0,self.size_row-1,n_pts))                                              #generating a list of points randomly from a unifrom distribution for x and y
        rand_y=np.round(np.random.uniform(0,self.size_col-1,n_pts))

        for i in range(0,n_pts):
            tempx=int(rand_x[i])                                                                                 #Picking our x and y from the list
            tempy=int(rand_y[i])
            if(self.map_array[tempx][tempy]==0):                                                                 #If the coordinates generated are obstacles in the map array then we have our first point
                rand_x2=int(np.round(np.random.normal(tempx,20)))                                                #generating points randomly from a gaussian/normal distribution for x and y
                rand_y2=int(np.round(np.random.normal(tempy,20)))
                if(rand_x2<self.size_row and rand_y2<self.size_col and self.map_array[rand_x2][rand_y2]==0):     #If the coordinates generated are witin limits and not obstacles in the map array then we have our second point
                    bridge_x=int(np.round((tempx+rand_x2)/2))                                                    #finding the midpoint of point1 and 2
                    bridge_y=int(np.round((tempy+rand_y2)/2))
                    if(self.map_array[bridge_x][bridge_y]!=0):                                                   #if the midpoint is not an obstacle in the map array we add it to the samples list
                        self.samples.append([bridge_x,bridge_y])

        


        
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":           #r is the radius around a node that we use to find pairs
            self.uniform_sample(n_pts)             #t is the threshold distance for connection, changes have been made in the template to reflect this in the search function.
            r=15                                   #changes have been made in main.py as well in the function calls.
            t=10
        elif sampling_method == "random":
            self.random_sample(n_pts)
            r=20
            t=15
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
            r=25
            t=75
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)
            r=30
            t=80

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]
        
        pairs = []
        pos=self.samples
        kdtree=spatial.KDTree(pos)                  #KDTree is made using the samples
        tempp=list(kdtree.query_pairs(r))           #Pairs found from the KDtree that have a max distance of r between them.

        for i in range(0,len(tempp)):
            p1=self.samples[tempp[i][0]]            #The 2 points in a pair are extracted from the pair list
            p2=self.samples[tempp[i][1]]
            colli=self.check_collision(p1,p2)       #collision check to see if there is an obstacle in the line connecting them
            if(colli==False):
                dist=self.dis(p1,p2)                #if there is no collision then distance between the points is found
                if(dist!=0):
                    pi1=tempp[i][0]                 #index of the points of the pair in the samples list is taken
                    pi2=tempp[i][1]
                    pairs.append((pi1,pi2,dist))    #appended to the pairs list in the necessary form


        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from([])
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))
        return t

    def search(self, start, goal, t):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]
        
        start_pairs = []
        for i in range(0,len(self.samples)-2):              #connecting the start node to the samples
            temp=self.samples[i]
            if(self.check_collision(start,temp)==False):
                dist=self.dis(start,temp)
                if(dist!=0 and dist<t):                     #if there are no obstacles between them, if the sample is not the start point itself and if the distance between them is lower than the threshold.
                    start_pairs.append(('start',i,dist))    #the sample is added to the start pair list



        goal_pairs = []
        for i in range(0,len(self.samples)-1):              #similarly for the goal node
            temp=self.samples[i]
            if(self.check_collision(goal,temp)==False):
                dist=self.dis(goal,temp)
                if(dist!=0 and dist<t):
                    start_pairs.append(('goal',i,dist))


        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
        