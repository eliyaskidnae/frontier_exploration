# import neccesry packges
import numpy as np 
from matplotlib import pyplot as plt 
from PIL import Image 
from math import sqrt
import random
# from DubinsPath import dubins_path_planner
# import DubinsPath as dubins_path_planner
import copy 
import math
import time
# class 
class Node:
    def __init__(self , x , y  , yaw = 0):
        self.x = x # x-position
        self.y = y# y-postion
        self.yaw = yaw # yaw angle
        self.path_x = [] # x-position of the path
        self.path_y = [] # y-position of the path
        self.path_yaw= [] # yaw angle of the path
        self.id = 0 # vertices id 
        self.f_score = float('inf') # initialize as inifinity 
        self.cost = float('inf') # initialize as inifinity 
        self.parent  = None
       
    def calcu_huristic(self,target):
        distance = sqrt( (self.x-target.x)**2 + (self.y-target.y)**2 )
        return distance   
    def get_distance(self,target):
        distance = sqrt( (self.x-target.x)**2 + (self.y-target.y)**2 )
        return distance 
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d
    def find_nearest_node(self,nodes):
        min_distance = float('inf')
        nearest_node = None
        for node in nodes:
            distance = self.get_distance(node)
            if(distance < min_distance):
                nearest_node = node
                min_distance = distance
        return nearest_node
    def find_nodes_with_in_radius(self,nodes,radius):
        # print(nodes)
        nodes_with_in_radius =[]
        for node in nodes:
            distance = self.get_distance(node)
            # less than or equal to the radius and the node is not the same as the current node
            if(distance <= radius and self != node):
              nodes_with_in_radius.append(node)
        return nodes_with_in_radius
    def find_costopt_nearest_node(self,nodes ,radius):
        # filter a nodes with a given radius 
        nodes_with_in_radius = self.find_nodes_with_in_radius(nodes,radius)
        if(not nodes_with_in_radius): # return the nearest node 
            return self.find_nearest_node(nodes)
        else : 
            # find a node with minimum cost from start to the given node
            optimam_node = self.find_nearest_node(nodes) 
            min_cost = float("inf")
            # print(nodes_with_in_radius)
            for node in  nodes_with_in_radius:
                cost = node.cost + self.get_distance(node)
                if(cost < min_cost):
                    optimam_node = node
                    min_cost = cost
            return optimam_node 
    def __str__(self):
        return str(self.id)
    # def __eq__(self,other):
    #     print("other" , other)
    #     return self.id == other.id
    
    
class RRT_Planner:
    def __init__(self,svc,k,q,p,dominion = [-10,10,-10,10] ,max_time = 10, is_RRT_star = True):
        self.svc      = svc
        self.k      = k 
        self.q      = q 
        self.p      = p 
        # self.dominion = dominion
        self.dominion = [-7,7,-7,7]
        self.vertices = []
        self.edges = []
        self.node_counter = 1
        self.path = []
        self.smoothed_path = []
        self.is_RRT_star = True # by deafault it is False we implement RRT
        self.radius = 20
        self.curvature = 0.05
        self.connect_circle_dist= 5.0,
        self.goal_xy_th = 0.5
        self.goal_yaw_th = np.deg2rad(1.0)
        self.robot_radius = 2
        self.search_until_max_iter = True
        self.path_resolution = 0.5
        self.max_iter_time = 20
        self.goal_found = False
            
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
    def cost_optimal_parent(self,new_node , current_parent):
      
        near_inds = self.find_near_nodes(new_node)
        if(not near_inds): # return the nearest node
            return new_node , current_parent
        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.vertices[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.svc.check_path(
                    [[t_node.x , t_node.y] , [near_node.x , near_node.y]]):
                # costs.append(self.calc_new_cost(near_node, new_node))
                costs.append(self.calc_new_cost(t_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            # print("There is no good path.(min_cost is inf)")
            return new_node ,current_parent

        min_ind = near_inds[costs.index(min_cost)]
        near_node =self.vertices[min_ind]
        new_node = self.steer(self.vertices[min_ind], new_node)
        new_node.cost = min_cost
        
        
        return new_node , near_node
    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.vertices):
            if node.get_distance(self.goal) <= self.goal_xy_th:
                goal_indexes.append(i)

        # angle check
        final_goal_indexes = goal_indexes
        # for i in goal_indexes:
        #     if abs(self.vertices[i].yaw - self.goal.yaw) <= self.goal_yaw_th:
        #         final_goal_indexes.append(i)

        if not final_goal_indexes:
            return None

        min_cost = min([self.vertices[i].cost for i in final_goal_indexes])
        for i in final_goal_indexes:
            if self.vertices[i].cost == min_cost:
                return i

        return None

    def rewire(self,new_node):
        near_inds = self.find_near_nodes(new_node)
        for i in near_inds:
            near_node = self.vertices[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.svc.check_path(
                [[edge_node.x,edge_node.y] , [new_node.x,new_node.y]])
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                for node in self.vertices:
                    if node.parent == self.vertices[i]:
                        node.parent = edge_node
                self.vertices[i] = edge_node
                self.propagate_cost_to_leaves(self.vertices[i])

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.vertices) + 1
       
        r = 50 * math.sqrt(math.log(nnode) / nnode)
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'q'):
            r = min(r, self.q)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.vertices]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds
    
    def propagate_cost_to_leaves(self, parent_node):

        for node in self.vertices:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)
    def smooth_path(self):
        path = self.path.copy()
        i = 0
        while i < len(path) - 2:
            j = len(path) - 1
            while j > i + 1:
                if self.svc.check_path([[path[i].x , path[i].y], [path[j].x , path[j].y]]):
                    # Remove nodes in between
                    del path[i+1:j]
                    j = i + 1  # Break the inner loop and continue with the next i
                else:
                    j -= 1
            i += 1
        self.smoothed_path = path
       
        path =[[n.x,n.y] for n in self.smoothed_path]
        # print("smoothed path cost" , path)
        return path
    def calc_new_cost(self, from_node, to_node):

        cost = from_node.cost + from_node.get_distance(to_node) 
        return cost   
    def Near_Vertices(self , qrand):
        qnear =  qrand.find_nearest_node(self.vertices )
        return qnear
    
    def wrap_angle(self,angle):
        return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

    def reconstract_path(self, goal_index):
        # print("final" , goal_index)
        self.path = [self.goal]
        
        node = self.vertices[goal_index]
        # node.cost = node.parent.cost + node.get_distance(node.parent)
        # print("goal_index" , goal_index , node.x , node.y)
        while node.parent:          
            node = node.parent
            self.path.append(node)
   
        # print("final_path cost" , self.path[0].cost)
        self.path.reverse()
        path =[[n.x,n.y] for n in self.path]
        # print("final_path " , path)
        
        return path
    def calc_distance_and_angle(self , from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
    def get_tree(self):
        tree_list = [[[edge[0].x , edge[0].y] ,[edge[1].x , edge[1].y]] for edge in self.edges]
        return tree_list
    def steer(self, from_node, to_node):
        extend_length= self.q

        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        # new_node.path_x = [new_node.x]
        # new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node
    def compute_path(self ,start , goal ):
        self.start = Node(start[0],start[1])   
        self.goal =  Node(goal[0],goal[1])
        last_time = time.time()
        self.vertices.append(self.start) # initialize the vertices list 
        self.start.cost = 0
        self.start.f_score = self.start.cost + self.start.calcu_huristic(self.goal)
        
        for k in range(self.k):
            qrand = self.Rand_Config()         
            while(not ([qrand.x,qrand.y])):
               qrand = self.Rand_Config()              
            qnear = self.Near_Vertices(qrand)
            qnew  = self.steer(qnear,qrand)
            if( qnew and self.svc.is_valid([qrand.x,qrand.y]) and self.svc.check_path([[qnear.x ,qnear.y ] ,[qnew.x , qnew.y] ]) ):         
                if(self.is_RRT_star == True): # if is RRT star is false we implemet cost_optimal_parent function                       
                  qnew , qnear = self.cost_optimal_parent(qnew, qnear)
                self.vertices.append(qnew)
                self.edges.append((qnear,qnew))
                qnew.parent = qnear 
               
                qnew.cost = qnear.cost + qnew.get_distance(qnear)
           
                if(self.is_RRT_star == True): # if is RRT star is true we implemet rewire function
                    self.rewire(qnew)
   
                if qnew.get_distance(self.goal) <= self.goal_xy_th :
                    self.goal_found = True
                    
                if(self.goal_found and (time.time()- last_time ) > self.max_iter_time) :
                        print("Time out")
                        last_index = self.search_best_goal_node()
                        if last_index is not None:
                        
                            # self.reconstract_path()
                            path = self.reconstract_path(last_index)
                            return self.smooth_path(),self.get_tree()
                print("Iteration" , k , "Time" , (time.time()- last_time ))
        last_index = self.search_best_goal_node()
                
        if last_index is not None:
      
            path = self.reconstract_path(last_index)
            return self.smooth_path() , self.get_tree()
        return [],[]
                
