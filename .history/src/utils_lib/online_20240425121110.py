import numpy as np

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

def distance_between_points(point1, point2):
    """
        Calculates the Euclidean distance between two points

        Args:
        - point1: list
            Position of the first point
        - point2: list
            Position of the second point

        Returns:
        - distance
            Euclidean distance between point1 and point2
        """
    return ((point1[0] - point2[0])** 2 + (point1[1] - point2[1])**2)**0.5

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1, is_unknown_valid=True):
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
        # for discretization of the path
        self.mindistance=0.05                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid
        self.invalid_points = []   
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True

        for i in range(len(self.invalid_points)):
            # print("Obstacle position added")
            self.addinvalid(self.invalid_points[i])
    
    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose): 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as unknown.
        pose_in_map=self.__position_to_map__(pose)
        copied_map=self.map.copy()

        if pose_in_map is None:
            return self.is_unknown_valid
        
        if self.is_unknown_valid:
            copied_map[copied_map == -1] = 0

        offset=self.distance/(self.resolution)
        row_i=int(pose_in_map[0]-offset)
        row_j=int(pose_in_map[0]+offset)
        col_i=int(pose_in_map[1]-offset)
        col_j=int(pose_in_map[1]+offset)
        valid=np.all(copied_map[row_i:row_j, col_i:col_j]==0)
        return valid

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path):
        # Iterate over all the waypoints in the path except the last point
        # Take the segment formed by 2 adjacent points and discretize it using self.mindistance
        # Check for collisions using the isvalid method
        for position_index in range(len(path)-1):
            waypoint1=list(path[position_index])
            waypoint2=list(path[position_index+1])
            waypoint1 = np.array(waypoint1)
            waypoint2 = np.array(waypoint2)

            distance_btwn_waypoints= (np.linalg.norm(np.array(waypoint2)-np.array(waypoint1)))
            theta= np.arctan2(waypoint2[1]-waypoint1[1],waypoint2[0]-waypoint1[0])
            # print(waypoint1,waypoint2)
            iter=int(distance_btwn_waypoints/(self.mindistance))
            # print(distance_btwn_waypoints)
            for i in range(iter):
                waypoint1[0]= (waypoint1[0]+ (self.mindistance* np.cos(theta)))
                waypoint1[1]= (waypoint1[1]+ (self.mindistance* np.sin(theta)))
                if self.is_valid(waypoint1):
                    continue
                else:
                    return False
        return True

            

    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):
        p=np.array(p) 
        cell=((p-self.origin)/self.resolution).astype(int)
        if np.any(cell < 0) or np.any(cell >= self.map.shape):
            return None
        return cell
    
    def addinvalid(self, position):
        map_pos=self.__position_to_map__(position)
        self.map[map_pos[0], map_pos[1]] = 99

    def set_as_invalid(self, position):
        self.invalid_points.append(position)
        self.addinvalid(position)

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
    

class Planner2:
    def  __init__(self, state_validity_checker, start, goal, prob_to_goal=0.4, del_q=5, max_iterations=10000, dominion=[-10, 10, -10, 10]):
        # define constructor ...
        self.max_iteration=max_iterations
        self.goal=[goal[0],goal[1]]
        self.state_validity=state_validity_checker
        self.tree_nodes=[]
        self.edges=[]
        self.path_dis=0
        self.distance=np.inf
        self.smooth_path_dis=np.inf
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

    def remove_edge(self,e):
        """
        Removes an edge from the RRT tree.

        Args:
        - e: tuple
            Edge to be removed (tuple of two nodes).
        """
        try:
            self.edges.remove(e)
        except:
            print(f"{e[0],e[1]} not in the edge list")

    
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
    
    def q_near(self,q_rand):
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
    
    def q_nearest(self, q_rand, n):
        """
        Finds the nodes that are less than n distance from a given point q_rand.

        Args:
        - q_rand: TreeNode
            Randomly sampled node for which nearest neighbors are to be found.
        - n: int
            Distance threshold for considering a node as a neighbour

        Returns:
        - list[TreeNode]
            List of neighbors to q_rand.
        """
        qnear=[]
        for vertex in self.tree_nodes:
            distance_from_vertex_to_new_point=self.dist(vertex.pos,q_rand.pos)
            if distance_from_vertex_to_new_point<n:
                qnear.append(vertex)
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
            if (q_rand==self.goal):
                row=q_rand[0]
                col=q_rand[1]
            else:
                row=round(q_rand[0],2)
                col=round(q_rand[1],2)
            q_new=tree_node(q_near,(row,col),q_near.cost+self.dist(q_near.pos,q_rand))
            return q_new

        theta= np.arctan2(q_rand[1]-q_near.pos[1],q_rand[0]-q_near.pos[0])
        row= round(q_near.pos[0]+ del_q* np.cos(theta),2)
        col= round(q_near.pos[1]+ del_q* np.sin(theta), 2)
        q_new= tree_node(q_near,(row,col), q_near.cost +self.dist(q_near.pos,(row,col)))
        return q_new
    
    def dist(self, q1,q2):
        return(np.linalg.norm(np.array(q1)-np.array(q2)))
    
    def compute_path(self):
        # Implement RRT algorithm.
        # Use the state_validity_checker object to see if a position is valid or not.
        self.goal_reached=False
        q_start=tree_node(None, self.start, 0)
        self.add_node(q_start)

        for iter in range(self.max_iteration):
            print("Iteration", iter)
            q_rand= self.q_random(self.prob_to_goal)
            q_near= self.q_near(q_rand)
            q_new = self.extend_tree(q_near,q_rand,self.del_q)

            if self.state_validity.check_path([q_near.pos,q_new.pos]):
                q_min=q_near
                q_near_list=self.q_nearest(q_new,10)
                # Find the node with least resistance towards start
                for q in q_near_list:
                    if self.state_validity.check_path([q.pos,q_new.pos]):
                        qcost=q.cost +  self.dist(q.pos,q_new.pos)
                        if qcost<q_new.cost:
                            # Update the new q with parent and cost found
                            q_min=q
                            q_new.parent=q_min
                            q_new.cost=q_min.cost +  self.dist(q_min.pos,q_new.pos)

                self.add_node(q_new)
                self.add_edge(q_min,q_new)

                # Find nodes which can be reached with q_new at less cost
                for q in q_near_list:
                    if ((q!=q_min) and (self.state_validity.check_path([q_new.pos,q.pos])) and (q.cost> q_new.cost + self.dist(q_new.pos,q.pos))):
                        self.remove_edge((q.parent,q))
                        q.parent=q_new
                        q.cost=q_new.cost + self.dist(q_new.pos,q.pos)
                        self.add_edge(q_new,q)
                # If goal is reached change the goal_reached value
                if q_new.pos[0]==self.goal[0] and q_new.pos[1]==self.goal[1] and self.goal_reached==False:
                    self.goal_reached=True
                    print(f"Goal reached in {iter} iterations!!!!")

            else:
                continue

        # Generate path
        if self.goal_reached:
            self.generate_path()
            # self._smooth_path()
            # print(self.path)

        else: 
            print("Goal is not reached!! Please generate a new tree or increase the number of iterations.")
        
        return self.path, []

    def generate_path(self):
        """
        Generates a path from the RRT* tree if the goal_reached is True. It uses the tree structure stored in the nodes
        and edges to find out the path.

        Returns:
        - None
        """
        goal_index=None
        
        goal_vertex=self.tree_nodes[0].pos
        goal_index=None

        for count,i in enumerate(self.tree_nodes):
            if (self.dist(i.pos,self.goal) < self.dist(goal_vertex,self.goal)) and self.state_validity.check_path([self.goal,i.parent.pos]):
                goal_index=count
                goal_vertex=i.pos
        
        if (goal_index!=None):
            self.tree_nodes[goal_index].pos=self.goal
            current_q=self.tree_nodes[goal_index]

        self.distance=0
        
        if goal_index!=None:
            while current_q!=self.tree_nodes[0]:
                self.path.append(current_q.pos)
                current_q=current_q.parent
            if current_q==self.tree_nodes[0]:
                self.path.append(current_q.pos)
            for i in range(len(self.path)-1):
                self.distance+=self.dist(self.path[i],self.path[i+1])
            print(f"Path Distance : {self.distance}")
            self.path=list(reversed(self.path))
            print(self.path)
        else: 
            print("Goal is not reached!! Please generate a new tree or increase the number of iterations.")

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
                        self.smooth_path_dis+=self.dist(current_pos,i)
                        current_pos=i
                        current_index=self.path.index(current_pos)
                        break
        self.smooth_path=list(reversed(self.smooth_path))


# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, state_validity_checker, dominion, max_iterations=1000):
    path_planner=Planner2(state_validity_checker,start_p,goal_p, max_iterations=max_iterations,dominion=dominion)
    Path=path_planner.compute_path()
    return Path


# Controller: Given the current position and the goal position, this function computes the desired 
# linear velocity and angular velocity to be applied in order to reach the goal.
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    
    # TODO: Use a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # To avoid strange curves, first correct the orientation and then the distance. 
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    # This function should return only  linear velocity (v) and angular velocity (w)
    psi_d=np.arctan2(goal[1] - current[1], goal[0] - current[0])

    if abs(wrap_angle(psi_d-current[2])) < 0.1:
        distance=distance_between_points(current,goal)
    else:
        distance=0
    
    w=Kw * wrap_angle (psi_d-current[2])
    v=Kv * distance
    return v, w
