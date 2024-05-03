import numpy as np
import math
import random
# from  utils_lib.rrt_modle import RRT
from  utils_lib.rrt_star import RRT_Planner
# from utils_lib.rrt import RRT
import numpy as np
from time import time


def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.2, is_unknown_valid=True , is_rrt_star = True):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid  
        self.is_rrt_star = is_rrt_star  
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        self.height = data.shape[0]
        self.width = data.shape[1]

    # computes distance between two positions 
    def get_distance( self , first_pose , second_pose):
        return math.sqrt((second_pose[0] - first_pose[0] )**2 + (second_pose[1] - first_pose[1]) **2)
    
    # Given a pose, returs true if the pose is not in collision and false othewise.

    def is_valid(self, pose): 
        # TODO: convert world robot position to map coordinates using method __position_to_map__
        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as unknown.
        m = self.__position_to_map__(pose)
        grid_distance = int(self.distance/self.resolution)
        # add 6 points upper and to lower limit 
        lower_x , lower_y = m[0] - grid_distance , m[1] - grid_distance  
        for lx in range(0,2*grid_distance):
            for ly in range(0,2*grid_distance):
                pose = lower_x + lx, lower_y + ly              
                # if one of the position is not free return False  , stop the loop 
                if(self.is_onmap(pose)):                           
                    if(not self.is_free(pose)): 
                        return False     
                # if  position is not in the map bounds return  is_unknown_valid
                else:
                    if(not self.is_unknown_valid):
                        return False
        return True
    def min_dis_obstacle(self, p, distance=1):
        
        m = self.__position_to_map__(p)
        grid_distance = int(distance/self.resolution)
        # print(m, grid_distance)
        # add 6 points upper and to lower limit 
        lower_x , lower_y = m[0] - grid_distance, m[1] - grid_distance
        min_dis = float('inf')
        for lx in range(2*grid_distance):
            for ly in range(2*grid_distance):
                pose = lower_x + lx, lower_y + ly
                
                if(not self.is_onmap(pose) or not self.is_free(pose)):
                    distance =  np.linalg.norm(np.array(pose) - np.array(m)) 
                    
                    min_dis = min(min_dis, distance)
        
        return   min_dis

    

    def not_valid_pose(self, pose): 
        # returns pose that are not valid
        m = self.__position_to_map__(pose)
        grid_distance = int(self.distance/self.resolution)
        # add 6 points upper and to lower limit 
        lower_x , lower_y = m[0] - grid_distance , m[1] - grid_distance  
        for lx in range(0,2*grid_distance):
            for ly in range(0,2*grid_distance):
                pose = lower_x + lx, lower_y + ly    
                # if(self.map[pose[0],pose[1]] >50):
                #     return pose
                # if one of the position is not free return False  , stop the loop 
                if(self.is_onmap(pose)):                           
                    if(not self.is_free(pose)): 
                        return pose     
                # if  position is not in the map bounds return  is_unknown_valid
                else:
                    if(not self.is_unknown_valid):
                        return pose
        return None

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path):
        step_size = 0.5*self.distance
        valid = True
        # TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True. 
        for index in range(len(path)-1):

            first_pose  = path[index] # initial path 
            second_pose = path[index+1] # next path 
            # direction vector from first_pose to second_pose
            dir_vector = np.array([second_pose[0] - first_pose[0] , second_pose[1] - first_pose[1]])
            distance = self.get_distance(first_pose , second_pose) # distance from first_pose to second_pose
            
            if distance == 0:
                norm_vect = np.array([0, 0])
            else:
                norm_vect = dir_vector / distance # Compute normal vector b/n two points
            discrtized_seg = np.array([first_pose]) # array of discritized segment 
            current = first_pose

            # while the distance b/n two poses is smaller than min distance 
            while(self.get_distance(second_pose , current) > step_size):          
                current  = current + norm_vect*step_size
                valid = self.is_valid(current)      
                # if one of the discritized poses are not vlid return false    
                if(not valid):
                    return False      
            # Finnally Check the goal point
            valid = self.is_valid(second_pose)

        # If each path is valid return true
        return valid
    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):      
        x , y  =  p # get x and y positions 
        m_x    =  (x - self.origin[0])/self.resolution  # x cell cordinate 
        m_y    =  (y - self.origin[1])/self.resolution  # y cell cordinate 
        return [round(m_x), round(m_y)]   
    def __map_to_position__(self, m):
        x ,y = m  
        p_x  = self.origin[0]+ x * self.resolution 
        p_y  = self.origin[1] + y * self.resolution
        return [p_x, p_y]
    
    def is_onmap(self,pose):    
        # checks if a given pose in grid is on the map 
        # if is is on the map return Truem else return is False 
        if( 0<=pose[0]< self.height and 0<= pose[1] < self.width):
            return True        
        else:
            return False
        # checks a given pose in grid is free  
    def is_free(self, pose): 
        # if is is free return True , which means  
        if self.map[pose[0],pose[1]] == 0 :
            return True, 
        #if it is unkown return opposite of is unkown valid 
        elif self.map[pose[0],pose [1]] == -1 : # return opposite of is unkown valid 
            return  self.is_unknown_valid
        # return false , if is is obstacle 
        return False   


# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, svc, bounds , max_time=7.0):
    # call rrt class to compute the path , which is imported from othe file
    print("computing path---")
    rrt = RRT_Planner(svc ,100 ,3, 0.2 , bounds, max_time )
    # returns the smooth path and the tree list
    path  , tree_list = rrt.compute_path(start_p, goal_p )
    # path = rrt.compute_path( start_p , goal_p)
    return path , tree_list

# Controller: Given the current position and the goal position, this function computes the desired 


def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    d = ((goal[0] - current[0])**2 + (goal[1] - current[1])**2)**0.5
    psi_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
    psi = wrap_angle(psi_d - current[2])

    v = 0.0 if abs(psi) > 0.05 else Kv * d
    w = Kw * psi
    return v, w
def move_to_point_smooth(current, goal, Kp=10, Ki=10, Kd=10, dt=0.05):
    # Compute distance and angle to goal
   
    dx = goal[0] - current[0]
    dy = goal[1] - current[1]
    dist = math.sqrt(dx**2 + dy**2)
    angle = wrap_angle(math.atan2(dy, dx) - current[2])

    # Compute errors
    error_dist = dist
    error_angle = angle

    # Store previous errors
    if 'prev_error_dist' not in move_to_point_smooth.__dict__:
        move_to_point_smooth.prev_error_dist = error_dist
    if 'prev_error_angle' not in move_to_point_smooth.__dict__:
        move_to_point_smooth.prev_error_angle = error_angle

    # Compute PID terms
    
    error_dist_deriv = (error_dist - move_to_point_smooth.prev_error_dist)
    error_angle_deriv = (error_angle - move_to_point_smooth.prev_error_angle)
    error_dist_integral = (error_dist + move_to_point_smooth.prev_error_dist)
    error_angle_integral = (error_angle + move_to_point_smooth.prev_error_angle)

    v = Kp * error_dist + Ki * error_dist_integral + Kd * error_dist_deriv
    w = Kp * error_angle + Ki * error_angle_integral + Kd * error_angle_deriv

    # Update previous errors
    move_to_point_smooth.prev_error_dist = error_dist
    move_to_point_smooth.prev_error_angle = error_angle

    # # Limit angular velocity to avoid overshooting
    if abs(angle) > 0.2:
        v = 0
    
    return v, w

    

class tree_node:
    """
    Represents a node in a tree used for pathfinding algorithms.

    Attributes:
    - parent: TreeNode
        The parent node in the tree.
    - pos: tuple
        The position coordinates (x, y) of the node.
    """

    def __init__(self, parent:tuple, pos:tuple, cost=None) -> None:
        self.pos=pos
        self.parent=parent
        self.cost=cost

    def __eq__(self, __o: object) -> bool:
        """
        Checks if two TreeNode objects are equal based on their positions.

        Args:
        - other: object
            Another TreeNode object to compare.

        Returns:
        - bool
            True if positions are equal, False otherwise.
        """
        x1=np.array(self.pos)
        x2=np.array(__o.pos)
        if np.isclose(x1,x2,atol=0.01).all():
            return True
        else:
            return False

# Define Planner class (you can take code from Autonopmous Systems course!)
class Planner:
    def  __init__(self, state_validity_checker, start, goal, prob_to_goal=0.4, del_q=5, max_iterations=10000, dominion=[-10, 10, -10, 10]):
        # define constructor ...
        self.max_iteration=max_iterations
        self.goal=[goal[0],goal[1]]
        self.state_validity=state_validity_checker
        self.tree_nodes=[]
        self.edges=[]
        self.path_dis=0
        self.smooth_path_dis=0
        self.goal_reached=False
        self.prob_to_goal=prob_to_goal
        self.del_q=del_q
        self.path=[]
        self.smooth_path=[]
        self.dominion=dominion
        self.start=[start[0],start[1]]

    def add_node(self, q:tree_node):
        """
        Adds a node to the RRT tree.

        Args:
        - q: TreeNode
            Node to be added to the tree.
        """
        self.tree_nodes.append(q)

    def add_edge(self, q1,q2):
        """
        Adds an edge between two nodes in the RRT tree.

        Args:
        - q1: TreeNode
            First node for the edge.
        - q2: TreeNode
            Second node for the edge.
        """
        self.edges.append((q1,q2))
    
    def q_random(self, prob):
        """
        Generates a random point with a probability threshold.

        Args:
        - prob: float
            Probability threshold for choosing the goal as a random point.

        Returns:
        - List
            Randomly sampled position (x, y) coordinates.
        """
        x= np.random.uniform(0,1)
        if x<prob:
            q_rand= self.goal
        else:
            q_rand=[np.random.uniform(self.dominion[0],self.dominion[1]),np.random.uniform(self.dominion[2],self.dominion[3])]
        return q_rand
    
    def q_nearest(self,q_rand):
        """
        Finds the nearest node to a given point q_rand among the tree nodes.

        Args:
        - q_rand: TreeNode
            Randomly sampled node for which the nearest node is to be found.

        Returns:
        - TreeNode or None
            Nearest node to q_rand among the tree nodes, or None if tree_nodes is empty.
        """
        distance = np.inf
        qnear=None
        for vertex in self.tree_nodes:
            distance_from_vertex_to_rand_point=self.dist(vertex.pos,q_rand)
            if distance>distance_from_vertex_to_rand_point:
                qnear=vertex
                distance=distance_from_vertex_to_rand_point
        return qnear
    
    def extend_tree(self,q_near, q_rand, del_q):
        """
        Extends the RRT tree from a near node (q_near) towards a random node (q_rand) with a specified step size (del_q).

        Args:
        - q_near: TreeNode
            Node closest to the randomly sampled node.
        - q_rand: tuple
            Randomly sampled position (x, y) coordinates.
        - del_q: float
            Step size for extending the tree.

        Returns:
        - q_new: TreeNode
            New node added to the tree towards q_rand from q_near.
        """
        if (self.dist((q_near.pos[0],q_near.pos[1]),q_rand))<del_q:
            q_new=tree_node(q_near,q_rand)
            return q_new

        theta= np.arctan2(q_rand[1]-q_near.pos[1],q_rand[0]-q_near.pos[0])
        row= np.round((q_near.pos[0]+ del_q* np.cos(theta)),2)
        col= np.round((q_near.pos[1]+ del_q* np.sin(theta)),2)
        q_new= tree_node(q_near,(row,col))
        return q_new
    
    def dist(self, q1,q2):
        return(np.linalg.norm(np.array(q1)-np.array(q2)))
    
    def compute_path(self):
        # Implement RRT algorithm.
        # Use the state_validity_checker object to see if a position is valid or not.
        self.goal_reached=False
        q_start=tree_node(None, self.start)
        self.add_node(q_start)
        for iter in range(self.max_iteration):
            print("Iteration", iter)
            q_rand= self.q_random(self.prob_to_goal)
            q_near= self.q_nearest(q_rand)
            q_new = self.extend_tree(q_near,q_rand,self.del_q)

            if self.state_validity.check_path([q_near.pos,q_new.pos]):
                self.add_node(q_new)
                self.add_edge(q_near,q_new)
                if q_new.pos==self.goal:
                    self.goal_reached=True
                    print("Path found in ", iter, "iterations")
                    break
            else:
                continue

        # Generate path
        current_q=self.tree_nodes[-1]
        if self.goal_reached:
            while current_q!=self.tree_nodes[0]:
                self.path.append(current_q.pos)
                # self.path_dis+=self.dist(current_q.pos, current_q.parent.pos)
                current_q=current_q.parent
            if current_q==self.tree_nodes[0]:
                self.path.append(current_q.pos)
            self.path=list(reversed(self.path))
            # print("Distance", self.path_dis)
            self._smooth_path()
            # print(self.path)

        else: 
            print("Goal is not reached!! Please generate a new tree or increase the number of iterations.")
        
        return self.smooth_path

    def _smooth_path(self):
        # Method to smoothen the RRT path.
        if len(self.path)!=0:
            print("Smoothing paths")
            self.smooth_path.append(self.path[-1])
            current_pos=self.path[-1]
            current_index=self.path.index(current_pos)
            while (self.path[0] in self.smooth_path) == False:
                new_list=self.path[0:current_index]
                for i in new_list:
                    if (self.state_validity.check_path([i,current_pos])):
                        self.smooth_path.append(i)
                        # self.smooth_path_dis+=self.dist(current_pos,i)
                        current_pos=i
                        current_index=self.path.index(current_pos)
                        break
        self.smooth_path=list(reversed(self.smooth_path))
        # print("Smooth distance:", self.smooth_path_dis)
        # print(self.smooth_path)
        
        # return self.smooth_path
    
