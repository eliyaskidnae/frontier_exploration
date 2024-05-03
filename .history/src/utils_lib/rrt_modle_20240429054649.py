# import neccesry packges
import numpy as np 
from matplotlib import pyplot as plt 
from PIL import Image 
from math import sqrt
import time
import random
# class that represent a node in the RRT tree
class Node:
    def __init__(self , x , y  ):
        self.x = x # x-position
        self.y = y # y-postion
        self.id = 0 # vertices id 
        self.f_score = float('inf') # initialize as inifinity 
        self.g_score = float('inf') # initialize as inifinity 
        self.parent  = None
    # calculate the huristic value of the node
    def calcu_huristic(self,target):
        distance = sqrt( (self.x-target.x)**2 + (self.y-target.y)**2 )
        return distance 
    # calculate the distance between the current node and the target node
    def get_distance(self,target):
        distance = sqrt( (self.x-target.x)**2 + (self.y-target.y)**2 )
        return distance 
    # find the nearest node from the given nodes with in the radius for RRT* method
    def find_nearest_node(self,nodes ,target):
        min_distance = float('inf')
        nearest_node = None
        for node in nodes:
            distance = self.get_distance(node)
            # if(distance < min_distance and distance != 0 and node != target):
            if(distance < min_distance and distance != 0 and node != target):
                nearest_node = node
                min_distance = distance
            
        return nearest_node
    # filter the nodes with in the given radius used for RRT* method
    def find_nodes_with_in_radius(self,nodes,radius):
        nodes_with_in_radius =[]
        for node in nodes:      
            distance = self.get_distance(node)
            if(distance <= radius):
              nodes_with_in_radius.append(node)
        return nodes_with_in_radius
    def __str__(self):
        return str(self.id)
# class that represent the RRT tree
class RRT:
    def __init__(self,svc,k,q,p,dominion = [-10,10,-10,10] ,max_time = 0.1, is_RRT_star = True):
        print("is_RRT_star" , is_RRT_star)
        self.svc      = svc
        self.k        = 5000
        self.q        = q
        self.p        = p 
        # self.dominion = dominion
        self.dominion = [-7,7,-7,7]
        self.nodes_list = {}
        self.max      = max_time 
        self.vertices =    []
        self.edges    =      [ ]
        self.node_counter  =  1
        self.path          = [ ]
        self.smoothed_path = [ ]
        self.goal_index = []
        self.is_RRT_star = True # by deafault it is False we implement RRT
        self.radius = 10    # radius for RRT* search  method
        self.max_time = 3 # max time for the search
        self.goal_found = False

    # Finds th  optimal node parent  with in given radius 
    # used for RRT* Cost Functionality 
    def cost_optimal_parent(self,qnew,current_parent):
        # filter a nodes with a given radius 
        nodes_with_in_radius = qnew.find_nodes_with_in_radius(self.vertices,self.radius)
        best_cost = current_parent.g_score + qnew.get_distance(current_parent)
        best_parent = current_parent
        if(not nodes_with_in_radius): # return the nearest node
            return best_parent
        else :
            for node in nodes_with_in_radius:
                new_node_cost = node.g_score + qnew.get_distance(node)
                if(new_node_cost < best_cost and self.svc.check_path([[node.x,node.y],[qnew.x , qnew.y]])):
                    best_parent = node

            return best_parent

    def rewire(self,qnew):
        # filter a nodes with a given radius 
        nodes_with_in_radius = qnew.find_nodes_with_in_radius(self.vertices,self.radius)
        for node in  nodes_with_in_radius:
            new_node_cost = qnew.g_score + qnew.get_distance(node)      
        
            if(new_node_cost < node.g_score and self.svc.check_path([[node.x,node.y],[qnew.x , qnew.y]])):
                node.parent = qnew
                # self.edges.remove((node.parent,node))
                node.g_score = new_node_cost
                # self.propagate_cost_to_leaves(node)

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.vertices:
            if node.parent == parent_node:
                node.g_score = parent_node.g_score + node.get_distance(parent_node)
                self.propagate_cost_to_leaves(node)

    def Rand_Config(self):       
        
        prob = random.random() # generate a random number between 0 and 1
        # generate a random node with in the dominion 
        x = random.uniform(self.dominion[0],self.dominion[1])
        y = random.uniform(self.dominion[2],self.dominion[3])
        qrand = Node(x,y)
        # if the random number is less than the probability of selecting the goal
        if(prob < self.p):          
           qrand = self.goal # set initialy qrand as goal

        return qrand            
    def Near_Vertices(self , qrand):
        qnear =  qrand.find_nearest_node(self.vertices , self.goal) # find the nearest node from the vertices
        return qnear
    def New_Config(self , qnear , qrand):
        dir_vector = np.array([qrand.x -  qnear.x , qrand.y - qnear.y])
        length = qnear.get_distance(qrand)
        if(length == 0):
            return qrand
        norm_vector = dir_vector/length

        if(self.q > length):
            return qrand
        qnew = np.array([qnear.x,qnear.y]) + norm_vector*self.q      
        qnew = Node(qnew[0] , qnew[1])
        return qnew
    
    def reconstract_path(self):
        v =[(n.x,n.y) for n in self.vertices]
        print("reconstracting path", v)
        current = self.goal
        self.path = [current]
        while( current != self.start):
            print("current",current.x , current.y)
            current = current.parent
            self.path.append(current)
        path =[(n.x,n.y) for n in self.path]
        self.path.reverse()
        path =[[n.x,n.y] for n in self.path]
        
        return path
    def smooth_paths(self): 
        if len(self.path)!=0:
            print("Smoothing paths")
            self.smoothed_path.append(self.path[-1])
            current_pos=self.path[-1]
            current_index=self.path.index(current_pos)
            while (self.path[0] in self.smoothed_path) == False:
                new_list=self.path[0:current_index]
                for i in new_list:
                    if (self.svc.check_path([[i.x , i.y],[current_pos.x  , current_pos.y]])):
                        self.smoothed_path.append(i)
                       
                        current_pos=i
                        current_index=self.path.index(current_pos)
                        break
        self.smoothed_path=list(reversed(self.smoothed_path))
        new_path =[(n.x,n.y) for n in  self.smoothed_path]
        x_path = [n.x for n in self.path]
        print("x_path",len(x_path))
        print("new path",len(new_path))
        return new_path
        # path = self.path
        # path.reverse()
        # first_point =path[0]
        # next_point = path[1]
        # new_path = [first_point]
        # for point in range(1, len(path)):
        #     np = path[point]
        #     if self.svc.check_path([[first_point.x , first_point.y], [np.x, np.y]]):
        #         next_point = path[point]
        #     else:
        #         new_path.append(next_point) 
        #         first_point = next_point 
        
        # new_path.append(path[-1])
        # new_path.reverse()
      
        # new_path =[(n.x,n.y) for n in new_path]
        # return new_path
    def smooth_path(self):
        print("smoothing path")
        counter = 0

        self.smoothed_path = [self.goal]
        while True:
            for node in self.path:
                next_path = self.smoothed_path[len(self.smoothed_path)-1]
                if(self.svc.check_path([[node.x , node.y],[next_path.x, next_path.y]])):
    
                    self.smoothed_path.append(node)
                    break
            if self.smoothed_path[len(self.smoothed_path)-1] == self.start:
                break
            
        
        self.smoothed_path.reverse()
        path =[(n.x,n.y) for n in self.smoothed_path]
        
        return path
    
    # get the tree of the RRT
    def get_tree(self):
        tree_list = [[[edge[0].x , edge[0].y] ,[edge[1].x , edge[1].y]] for edge in self.edges]
        return tree_list
    def compute_path(self , start , goal):
        self.start_time = time.time()
        self.start = Node(start[0],start[1])   
        self.goal =  Node(goal[0],goal[1])
        self.node_list[self.start] = 0
    
        for k in range(self.k):
            qrand = self.Rand_Config()
            while(not self.svc.is_valid([qrand.x,qrand.y])):
               qrand = self.Rand_Config()
          
            qnear = self.Near_Vertices(qrand)
            qnew  = self.New_Config(qnear,qrand)
      

            if( self.svc.check_path([[qnear.x ,qnear.y ] ,[qnew.x , qnew.y] ])):    
                
                if(self.is_RRT_star == True): # if is RRT star is false we implemet Cost function
                     # We Select new parent with optimal cost
                     qnear = self.cost_optimal_parent(qnew,qnear)

                new_cost = self.node_list[qnear] + qnew.get_distance(qnear)
               
                if qnew not in self.node_list or self.node_list[qnew] > new_cost:
                    self.node_list[qnew] = new_cost
                    qnew.parent = qnear
                    self.edges.append((qnear,qnew))   
                    self.node_counter += 1     
                
       
                if(self.is_RRT_star == True): # if is RRT star is true we implemet rewire function
                    self.rewire(qnew)
                    pass

                if(qnew == self.goal):
                    self.goal_found = True
                    print("goal found")
                  
            
            if((time.time() - self.start_time) > self.max_time and not  self.goal_found ):

                self.max_time +
        if(self.goal_found):
            print("max iteration reached")
            self.reconstract_path()
            # self.smoothed_path
            return self.smooth_paths() , self.get_tree() 
                
        return [], self.get_tree()

    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.vertices):
            if node.get_distance(self.goal) <= self.goal_xy_th:
                goal_indexes.append(i)

        # angle check
        # final_goal_indexes = []
        # for i in goal_indexes:
        #     if abs(self.vertices[i].yaw - self.goal.yaw) <= self.goal_yaw_th:
        #         final_goal_indexes.append(i)

        if not goal_indexes:
            return None

        min_cost = min([self.vertices[i].cost for i in goal_indexes])
        for i in goal_indexes:
            if self.vertices[i].g_cost == min_cost:
                return i

        return None
# def generate_final_course(self, goal_index):
#         print("final")
#         path = [[self.goal.x, self.goal.y]]
        
#         node = self.vertices[goal_index]
#         print("goal_index" , goal_index , node.x , node.y)
#         while node.parent:
          
#                 path.append()
#                 # print("path" , path)
#             node = node.parent
#         path.append([self.start.x, self.start.y])
#         self.path = path
#         return path
