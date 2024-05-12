import math
import numpy as np
from scipy import interpolate
from collections import OrderedDict
from online_planning import StateValidityChecker as svc



tolerance = 0.1

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

def distance_to_goal( goal,current):        #distance between the current pose and the goal pose 
    return math.sqrt(( goal[0] - current[0])**2 + ( goal[1] - current[1])**2)

def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    v = Kv * distance_to_goal(current, goal)
    steering = wrap_angle(math.atan2(goal[1] - current[1], goal[0] - current[0]))
    w = Kw * wrap_angle((steering - current[2]))

    if abs(steering- current[2]) > 0.16:  #Check if heading is within a tolerance level of desired heading
        # Set heading towards goal
        return 0,w
    else:
        # If the heading is within tolerance, change both v and w
        # so unneccessary stopping for minor turns doesn't happen
        return v,w
    

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



 # Dynamic window approach algorithm to select best v and w
 
def dynamic_window_approach(self, goal, v_max, w_max, dv_resolution, dw_resolution, dt, t_total): 

          grid_map = np.array(self.svc.map)

          weight_range = [0, 1]

          # Initial state
          self.current_pose
          xi, yi, theta_i = self.current_pose[0], self.current_pose[1], self.current_pose[2]
          pose = [xi, yi, theta_i]
          self.dwa_heading(pose)

          vi = 0

          goal = goal # m
          goal_theta = self.svc.compute_angle((xi, yi), goal) # rad
          goal_theta = self.correct_angle(goal_theta)

          goal_heading = [xi, yi, goal_theta]
          self.dwa_goal_heading(goal_heading)

          thresh = np.pi/6    
          angle_diff = abs(self.correct_angle(theta_i) - goal_theta)

          no_of_branches = len(np.arange(-w_max, w_max, dw_resolution))
          collision_list = [False] * no_of_branches

          # Empty lists
          trajectory = []

          speed_dict = {}
          goal_direction_dict = {}
          collision_dict = {}

          xy_to_vw = {}
          xy_list = []

          for v in np.arange(0, v_max, dv_resolution):
            for j, w in enumerate(np.arange(-w_max, w_max, dw_resolution)):
                x, y, theta = xi, yi, theta_i

                # simulate motion for the given command
                motion = []
                for _ in np.arange(0, t_total, dt):
                    x, y, theta = self.simulate_motion(v, w, dt, x, y, theta)
                    x = round(x, 2)
                    y = round(y, 2)

                    motion.append((x, y))
                    p = self.svc.__position_to_map__([y, x])
                    
                    if p == [] or grid_map[p[1], p[0]] != 0:
                        collision_list[j-1] = True
                
                trajectory.append(motion)

                #if x != xi and y!= yi:
                xy_to_vw[(x, y)] = (v, w)
                xy_list.append([x, y])

                speed_cost = v - vi
                speed_dict[(x, y)] = speed_cost

                theta = self.correct_angle(theta)

                goal_direction_cost =  abs(goal_theta - theta)
                goal_direction_dict[(x, y)] = goal_direction_cost
            
          for index, line in enumerate(trajectory):
            k = index
            while k > no_of_branches:
                k -= no_of_branches
            
            if not collision_list[k-1]:
                x, y = zip(*line)
                #if x[-1] != xi and y[-1] != yi:
                collision_cost = weight_range[1]
                collision_dict[(x[-1], y[-1])] = collision_cost
          
          self.dwa_tree(xy_list)

          # Compute weights
          new_speed_dict = self.svc.scale(speed_dict, weight_range)
          new_goal_direction_dict = self.svc.scale(goal_direction_dict, [weight_range[1], weight_range[0]])
          
          # Find best command
          result_dict = self.svc.add_dicts(new_speed_dict, new_goal_direction_dict, collision_dict)
          key_with_max_value = max(result_dict, key=result_dict.get)
          
          best_v, best_w = xy_to_vw[key_with_max_value]
          self.best_v_w_marker(key_with_max_value)

          # If goal is out of the window, switch to normal controller
          if angle_diff > thresh:
              best_v, best_w = move_to_point(self.current_pose, self.path[0])
              best_v = round(best_v, 2)
              best_w = round(best_w, 2)
        
          print("The best (v, w) command is", [best_v, best_w])
          #best_v, best_w = 0, 0

          return best_v, best_w
      
      
      
      