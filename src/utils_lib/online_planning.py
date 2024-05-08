import numpy as np
import math
import random
from  utils_lib.rrt_modle import RRT
from  utils_lib.rrt_debiuns import RRT as RRT_DB
from utils_lib.rrt_module_debiun import RRT as RRT_Debiun
from  utils_lib.rrt_star import RRT_Planner
# from utils_lib.rrt import RRT
import numpy as np
from time import time
import scipy.spatial
import matplotlib.pyplot as plt
import numpy as np

local_map = None
local_origin = None
local_resolution = None

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
        global local_map, local_origin, local_resolution
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        self.height = data.shape[0]
        self.width = data.shape[1]
        local_map = data
        local_origin = origin
        local_resolution = resolution
        

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
    
    def check_path_smooth(self,paths):
        for path in paths:
            if(not self.is_valid(path)):

                return False
        
        return True
    
    
    
    def check_path(self, path):
        step_size = 0.2*self.distance
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
            valid = self.is_valid(first_pose)
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
    # rrt = RRT(svc ,2000 ,1, 0.2 , bounds, max_time )
    rrt = RRT_DB(svc ,1000    ,0.6, 0.2 , bounds, max_time )
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


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.v_max = 0.15  # [m/s]
        self.v_min = -0.15  # [m/s]
        # self.w_max = 40.0 * math.pi / 180.0  # [rad/s]
        self.w_max = 0.3  # [rad/s]
        self.max_lin_accel = 0#0.8  # [m/ss]
        self.max_ang_accel = 0  # [rad/ss]
        self.v_resolution = 0.01 #0.02  # [m/s]
        self.w_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for integrate_motion prediction
        # self.predict_time = 1.5  # [s]
        # self.heading_cost_weight = 0.8 #1.0
        # self.velocity_cost_weight = 0.1#1.0
        # self.clearance_cost_weight = 0.5#1.0
        # # self.dist_to_goal_cost_weight = 0.3
        # self.dist_to_goal_cost_weight = 0
        # self.gamma = 1.0  # for calculating dist_to_goal_cost
        # self.goal_threshold = 0.2  # [m] If the distance between the robot and the goal is less than this value, the goal is achieved.
        # self.robot_stuck_flag_cons = 0.01  # constant to prevent robot stucked



        # Also used to check if goal is reached in both types
        self.robot_radius = 0.6  # [m] for collision checking
        # map = np.zeros((14, 14))
        # map[1:4, 1:4] = 1
        # map[1:4, 10:13] = 1
        # map[10:13, 10:13] = 1
        # map[5:9, 5:9] = 1


        self.predict_time = 2.5  # [s]
        self.heading_cost_weight = 1 #1.0
        self.velocity_cost_weight = 0.8 # 1.0
        self.clearance_cost_weight = 0.15 #1.0
        # self.dist_to_goal_cost_weight = 0.3
        self.dist_to_goal_cost_weight = 0
        self.gamma = 1.0  # for calculating dist_to_goal_cost
        self.goal_threshold = 0.2  # [m] If the distance between the robot and the goal is less than this value, the goal is achieved.
        self.robot_stuck_flag_cons = 0.01  # constant to prevent robot stucked


def integrate_motion(state, v, w, dt):
    """
    this function return the next state based on current state, velocity and angular velocity
    
    :param state: current state [x(m), y(m), yaw(rad), v(m/s), w(rad/s)]
    :type state: list or numpy.array
    :param v: current velocity [m/s]
    :type v: float
    :param w: current angular velocity [rad/s]
    :type w: float
    :param dt: time tick [s]
    :type dt: float

    :return: next state
    :rtype: list or numpy.array
    """

    # state[2] += w * dt
    # state[0] += v * np.cos(state[2]) * dt
    # state[1] += v * np.sin(state[2]) * dt
    state[3] = v
    state[4] = w

    # New rotate and move simultaneously
    if abs(state[2]) == 0:
        state[0] += v * np.cos(state[2]) * dt
        state[1] += v * np.sin(state[2]) * dt
        state[2] += w * dt
    else:
        ratio = v/w
        a = wrap_angle(state[2] + w * dt)
        state[0] += -ratio * (np.sin(state[2]) - np.sin(a))
        state[1] += -ratio * (-np.cos(state[2]) + np.cos(a))
        state[2] += w * dt

    return state


def get_dynamic_window(state, config):
    """
    summary: this function return the dynamic window from robot specification
    it uses the current state and robot specification to determine the dynamic window

    :param state: current state [x(m), y(m), yaw(rad), v(m/s), w(rad/s)]
    :type state: list or numpy.array

    :return: dynamic window [v_min, v_max, w_min, w_max]
    :rtype: list
    """

    # Dynamic window from robot specification
    v_min = max(config.v_min, state[3] - config.max_lin_accel * config.dt)
    v_max = min(config.v_max, state[3] + config.max_lin_accel * config.dt)
    yaw_rate_min = max(-config.w_max,
                       state[4] - config.max_ang_accel * config.dt)
    yaw_rate_max = min(config.w_max,
                       state[4] + config.max_ang_accel * config.dt)
    dw = [v_min, v_max,
          yaw_rate_min, yaw_rate_max]

    return dw


def predict_trajectory(state, v, w, config):
    """
    summary: this function predicts the trajectory based on the current state, velocity and angular velocity

    :param state: current state [x(m), y(m), yaw(rad), v(m/s), w(rad/s)]
    :type state: list or numpy.array
    :param v: current velocity [m/s]
    :type v: float
    :param w: current angular velocity [rad/s]
    :type w: float
    :param config: configuration parameters
    :type config: class

    :return: predicted trajectory
    :rtype: numpy.array
    """

    x = np.array(state)
    trajectory = np.array(x)
    time = 0
    # print("x: ", x)
    while time <= config.predict_time:
        x = integrate_motion(x, v, w, config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_final_cost(trajectory, goal, obstacles, config):
    """
    summary: this function calculates the final cost of the trajectory

    :param trajectory: trajectory to evaluate
    :type trajectory: numpy.array
    :param goal: goal position [x(m), y(m)] 
    :type goal: list or numpy.array
    :param obstacles: obstacle positions [x(m) y(m), ....]
    :type obstacles: list of lists or numpy.array
    :param config: configuration parameters
    :type config: class

    :return: final cost
    :rtype: float
    """

    heading_cost = calc_heading_cost(trajectory, goal)
    velocity_cost = calc_velocity_cost(trajectory, config)
    clearance_cost = calc_clearance_cost(trajectory, obstacles, config)

    dist_to_goal = calc_dist_to_goal_cost(trajectory, goal)
    # print("heading_cost: ", heading_cost)
    # print("velocity_cost: ", velocity_cost)
    # print("clearance_cost: ", clearance_cost)
    # print("dist_to_goal: ", dist_to_goal)


    final_cost = config.gamma * (config.heading_cost_weight * heading_cost + config.velocity_cost_weight * velocity_cost + config.clearance_cost_weight * clearance_cost + config.dist_to_goal_cost_weight * dist_to_goal)

    return final_cost
    

def get_local_plan(x, dw, config, goal, ob):
    """
    summary: this function calculates the control input and trajectory based on the dynamic window

    :param x: current state [x(m), y(m), yaw(rad), v(m/s), w(rad/s)]
    :type x: list or numpy.array
    :param dw: dynamic window [v_min, v_max, w_min, w_max]
    :type dw: list
    :param config: configuration parameters
    :type config: class
    :param goal: goal position [x(m), y(m)]
    :type goal: list or numpy.array
    :param ob: obstacle positions [x(m) y(m), ....]
    :type ob: list of lists or numpy.array

    :return: control input
    :rtype: list
    :return: trajectory
    :rtype: numpy.array
    """

    state = x[:]
    min_cost = float("inf")
    best_control = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for w in np.arange(dw[2], dw[3], config.w_resolution):

            trajectory = predict_trajectory(state, v, w, config)
            final_cost = calc_final_cost(trajectory, goal, ob, config)

            # search minimum trajectory
            if final_cost <= min_cost:
                min_cost = final_cost
                best_control = [v, w]
                best_trajectory = trajectory
                if abs(best_control[0]) < config.robot_stuck_flag_cons and abs(x[3]) < config.robot_stuck_flag_cons:
                    # print("best_control: ", best_control)

                #     # to ensure the robot do not get stuck in
                #     # best v=0 m/s (in front of an obstacle) and
                #     # best omega=0 rad/s (heading to the goal with
                #     # angle difference of 0)
                    # best_control[1] = -config.max_ang_accel
                    best_control[0] = 0.03
    # if best_control[0] == 0.0:
    #     best_control[0] = 0.01

    if abs(best_control[0]) < config.robot_stuck_flag_cons: #and abs(x[3]) < config.robot_stuck_flag_cons:
        # print("best_control: ", best_control)

    #     # to ensure the robot do not get stuck in
    #     # best v=0 m/s (in front of an obstacle) and
    #     # best omega=0 rad/s (heading to the goal with
    #     # angle difference of 0)
        # best_control[1] = -config.max_ang_accel
        best_control[0] = config.v_max

    # if the velocity is negative (moving backwards), then the robot should turn around
    if best_control[0] < 0.0:
        best_control[0] = best_control[0] / 4.0
    return best_control, best_trajectory


def calc_clearance_cost(trajectory, obstacles, config):
    """
    summary: this function calculates the clearance cost, the minimum distance between the trajectory and the obstacles

    :param trajectory: trajectory to evaluate
    :type trajectory: numpy.array
    :param obstacles: obstacle positions [x(m) y(m), ....]
    :type obstacles: list of lists or numpy.array
    :param config: configuration parameters
    :type config: class

    :return: clearance cost
    :rtype: float
    """

    trajectory = trajectory[:, 0:2]
    dist = scipy.spatial.distance.cdist(trajectory, obstacles)

    if np.array(dist <= config.robot_radius).any():
        return float("Inf")

    min_dist = np.min(dist)
    return 1.0 / min_dist  # OK

def calc_velocity_cost(trajectory, config):
    """
    summary: this function calculates the velocity cost, the difference between the current velocity and the maximum velocity

    :param trajectory: trajectory to evaluate
    :type trajectory: numpy.array
    :param config: configuration parameters
    :type config: class

    :return: velocity cost
    :rtype: float
    """

    # velocity_cost = abs(config.v_max - trajectory[-1, 3]) / config.v_max
    velocity_cost = config.v_max - trajectory[-1, 3] 

    # vx =  trajectory[-1, 3] * np.cos(trajectory[:, 2])
    # vy =  trajectory[-1, 3] * np.sin(trajectory[:, 2])
    # v = np.sqrt(vx**2 + vy**2)
    # # #calculate the progress of the robot in the trajectory
    # # progress = v /  np.linalg.norm(trajectory[:, :2], axis=1)

    # # print("progress", progress)
    # # #take the average of the progress
    # # progress = np.mean(progress)

    # # #calculate the cost of the velocity
    # # velocity_cost = 1 - progress


    # distances = np.linalg.norm(np.diff(trajectory[:, :2], axis=0), axis=1)

    #     # Calculate the time it takes to travel between each pair of points in the trajectory
    # times = sum(distances) /  trajectory[-1, 3]
    # time = times - 1/ config.predict_time
    # return time

    return velocity_cost

def calc_dist_to_goal_cost(trajectory, goal):
    """
    summary: this function calculates the distance to goal cost, the distance between the current position and the goal

    :param trajectory: trajectory to evaluate
    :type trajectory: numpy.array
    :param goal: goal position [x(m), y(m)]
    :type goal: list or numpy.array

    :return: distance to goal cost
    :rtype: float
    """

    # dx = goal[0] - trajectory[-1, 0]
    # dy = goal[1] - trajectory[-1, 1]
    # dist = math.hypot(dx, dy)

    # Cost: Distance to goal (using error Angle)
    start_goal_vector = np.sqrt(goal[0]**2 + goal[1]**2)
    start_current_vector = np.sqrt(trajectory[-1][0]**2 + trajectory[-1][1]**2)
    distance = np.sqrt((trajectory[-1][0]-goal[0])**2 + (trajectory[-1][1]-goal[1])**2)
    dot_product = (goal[0] * trajectory[-1][0])+ (goal[1] * trajectory[-1][1])
    cos_theta = dot_product / (start_goal_vector * start_current_vector)
    theta = np.arccos(cos_theta)
    goal_cost = (distance*0.1 + theta)

    return goal_cost
    # return dist


def calc_heading_cost(trajectory, goal):
    """
    summary: this function calculates the heading cost, the difference between the current yaw and the yaw of the goal

    :param trajectory: trajectory to evaluate
    :type trajectory: numpy.array
    :param goal: goal position [x(m), y(m)]
    :type goal: list or numpy.array
    
    :return: heading cost
    :rtype: float
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    goal_yaw = math.atan2(dy, dx)
    cost_angle = goal_yaw - trajectory[-1, 2]
    #DWA requires the cost to be non-negative, so we cannot direcly use the angle difference
    #instead we use the absolute value of the angle difference

 
    heading_cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return heading_cost

config = Config()

def move_to_point_dw(x, goal):
    """
    summary: this function calculates the control input and trajectory based on the dynamic window

    :param x: current state [x(m), y(m), yaw(rad), v(m/s), w(rad/s)]
    :type x: list or numpy.array
    :param config: configuration parameters
    :type config: class
    :param goal: goal position [x(m), y(m)]
    :type goal: list or numpy.array
    :param obstacles: obstacle positions [x(m) y(m), ....]
    :type obstacles: list of lists or numpy.array

    :return: control input
    :rtype: list
    :return: trajectory
    :rtype: numpy.array
    """
    global config, local_map
    # print("goal", goal)

    # if x[3] <= 0.1:
        # x[3] = 0.1

    indices = np.where(local_map > 50)
    a = np.array([indices[0]])
    b = np.array([indices[1]])
    obstacles = np.hstack((a.T, b.T))
    # obstacles = obstacles * config.map_resolution + config.map_origin

    dw = get_dynamic_window(x, config)

    control, trajectory = get_local_plan(x, dw, config, goal, obstacles)

    print("control", control)
    #calculate the distance between the robot and the goal
    # distance = np.linalg.norm(np.array(goal) - np.array(x[:2]))

    return control, trajectory
