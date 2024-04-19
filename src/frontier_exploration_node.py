
#!/usr/bin/env python
from spatialmath import Quaternion
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
# from utils_lib.online_planning import StateValidityChecker
from utils_lib.online_planning import StateValidityChecker, move_to_point, compute_path , wrap_angle
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool
import tf
import math
import random
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from skimage import measure

import cv2
import copy

class FrontierExploration:
    def __init__(self):
                # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        self.frontier_map = []
        self.frontier_list = []
        self.clustered_frontier = []
        self.frontier_marker = MarkerArray()
        self.frontier_marker.markers = []
        self.clustered_marker = MarkerArray()
        self.clustered_marker.markers = []
        self.cluster_dist_the = [1,float("inf")]
        self.svc = StateValidityChecker(  ) 
        # self.frontier_exploration_pub = rospy.Publisher('/frontier_exploration', String, queue_size=10)
        # self.frontier_exploration_sub = rospy.Subscriber('/frontier_exploration', String, self.frontier_exploration_callback)
        self.frontier_pub = rospy.Publisher('/frontier', MarkerArray, queue_size=2)
        self.cluster_pub = rospy.Publisher('/clustered_frontiers', MarkerArray, queue_size=2)
        
        # SUBSCRIBERS
        rospy.Subscriber('/odom', Odometry, self.get_odom)
        rospy.Subscriber('/goal_reached', Bool, self.get_goal)
        rospy.Subscriber('/projected_map', OccupancyGrid, self.get_gridmap)
        
        self.move_goal_sub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
       
        self.robot_radius = 0.2
        self.goal_reached = True
        self.current_pose = None
        
    
    def get_goal(self, goal):
        ''' this function is called when the goal is reached'''
        print('Goal reached:',goal)
        if goal.data == True:
            self.goal_reached=True
        
    def get_gridmap(self, gridmap):
        
        # To avoid map update too often (change value '1' if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:            
            self.last_map_time = gridmap.header.stamp
            date = str(rospy.Time.now())
            # print('Map received at:', date)
            self.grid_map = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
           
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            resolution = gridmap.info.resolution
           
            [self.width,self.height] = self.grid_map.shape
            self.svc.set(self.grid_map, resolution, origin)
            grid_map2 = self.grid_map.copy()
            print(self.goal_reached )
            if  self.current_pose is not None:
                print('Exploring')
                
             
                # intialize frontier map
                self.dilate_obstacle() # increase the size of the obstacle to consider robot size
                self.frontier_map = np.zeros_like(self.grid_map)
                self.frontier_list = []
                self.publish_frontiers()
                # selec frontiers
                self.frontier_map , self.frontier_list = self.set_frontier_map()
                # cluster frontiers
                self.clustered_frontier = self.cluster_frontier()
                # select candidate frontier
                if( self.goal_reached ):
                    self.goal_reached = False
                    self.candidate_frontier()

            
    def set_frontier_map(self):

        ''' this function set the frontier map by considering the free space as frontier 
            if one of its neighbour is unknown or out of map'''
        
        direction = [[0, 1], [1, 0], [0, -1], [-1, 0]]
        free_space = np.where(self.grid_map == 0)
        free_space = list(zip(free_space[0], free_space[1]))
        
        # loop over all free space 
        for f in free_space:
            for d in direction:
                # consider a free space as frontier if one of its neighbour is unknown or out of map
                niebour = [f[0]+d[0],f[1]+d[1]]
                if not self.svc.is_onmap(niebour) or self.grid_map[niebour[0],niebour[1]] == -1 :
                    self.frontier_map[f[0]][f[1]] = 255
                    self.frontier_list.append(f)
                    break
       
        self.publish_frontiers()
        return self.frontier_map , self.frontier_list
       
        
    def cluster_frontier(self):
        ''' this function clusters the frontier to get the candidate frontiers'''

        self.labeled_frontier_map = measure.label(self.frontier_map ,  background=0)
        self.save_image()

        # Call regionprops function to get characteristics of each frontier
        # Example: size, orientation, centroid

        regions = measure.regionprops(self.labeled_frontier_map)
        self.clustered_frontier = []  
        grid = copy.deepcopy(self.grid_map)    
        grid[np.where(grid==50)]=127 # Unknown space
        grid[np.where(grid==0)]=255 # free space
        grid[np.where(grid==100)]=0 # occupied space
        cv2.imwrite("testing.png",grid)
        grid = cv2.imread("testing.png")
        i= 0
        # print('Number of frontiers:',regions)
        for region in regions:
            
            if region.area > 5.0:
                # get centroid of each frontier using regionprops property
                label = region.label
                connected_points = region.coords 
                median = int(len(connected_points)/2)
                centroid = connected_points[median]           
                x,y = centroid
                grid = cv2.circle(grid, (int(y),int(x)), 1, (0,0,255),2)
                cv2.imwrite("frontiercenter.png",grid)
                self.clustered_frontier.append([x,y,label])
                self.publish_clustered_frontiers()

        return  self.clustered_frontier
    
    def candidate_frontier(self):
        ''' this function selects the candidate frontier to explore'''
        
        distance_cost = self.get_clusters_by_distance(self.clustered_frontier)
        keys = list(distance_cost.keys())



        if len(keys)==0:
            return
        else:
            # get the frontier with minimum distance from the robot

            self.sorted_cluster = dict(sorted(distance_cost.items(), key=lambda item: item[1]))
            print("sorted_cluster",self.sorted_cluster)
            self.set_goal_point(self.sorted_cluster)
            # # get the frontier with higher area
            # area_cost = self.get_clusters_by_area(keys)

            # # get the frontier with higher density
            # density_cost = self.get_clusters_by_density(keys)

            # # get the frontier with robot orientation
            # orientation_cost = self.get_clusters_with_robot_orientation(keys)




            # calculate the cost of each cluster 

            # for key in self.sorted_cluster:
            #     distance = distance_cost[key]
            #     area = area_cost[key]
            #     density = density_cost[key]
            #     orientation = orientation_cost[key]
            #     cost = - distance + area + density - orientation

            #     self.sorted_cluster[key] = cost
            
     
    def set_goal_point(self, goal_point):

        ''' this function sets the goal point to move the robot'''

        for key in self.sorted_cluster:
            x,y,label = self.clustered_frontier[key]
            goal_point = [x,y]
            print("goal_point_map",key,goal_point)
            
            if(goal_point is not None and self.svc.is_valid(goal_point) ):
                   
                    goal_point = self.svc.__map_to_position__(goal_point)
                    print("goal_point_world",goal_point)
                    goal = PoseStamped()
                    goal.header.frame_id = "odom"
                    goal.header.stamp = rospy.Time.now()
                    goal.pose.position.x = goal_point[0]
                    goal.pose.position.y = goal_point[1]
                
                    self.move_goal_sub.publish(goal)
                    break

        def get_clusters_by_distance(self, centroids):
            ''' this function filters the frontier clusters with small distance from the robot'''
            distance_cost = {}
            out_range_cluster = {}
            i = 0
            for centroid in centroids:
                # get centroid of each frontier using regionprops property
                # centroid = region.centroid
                x,y,label = centroid
                # print("current_pose",self.current_pose)
                x_c, y_c = self.svc.__map_to_position__([x,y])
                x_r, y_r = self.current_pose[0:2]
                print("x_c,y_c",x_c,y_c)
                print("x_r,y_r",x_r,y_r)
                distance = np.linalg.norm([x_c-x_r,y_c-y_r])
                print("distance",distance)
                min_dis , max_dis = self.cluster_dist_the
                if(min_dis<distance<max_dis):
                  distance_cost[i] = distance
                
                else:
                    out_range_cluster[i] = distance
                i = i+1
            if len(distance_cost)>0:
                # print("distance_cost",distance_cost)
                return distance_cost
            else:
                # print("out_range_cluster",out_range_cluster)
                return out_range_cluster
        

    def get_clusters_by_size(self, cluster_keys):
        ''' this function filters the frontier clusters with higher area'''
        size_cost = {}
        
        for key in cluster_keys:
            # get centroid of each frontier using regionprops property
            # centroid = region.centroid
            x,y,label = self.clustered_frontier[key]
            
            region = measure.regionprops(self.labeled_frontier_map )[label]
            size = region.area
            size_cost[key] = size
            
        return size_cost
        pass

    def get_clusters_by_density(self, cluster_keys):
        ''' this function filters the frontier clusters with higher density'''
        density_cost = {}
        for key in cluster_keys:
            # get centroid of each frontier using regionprops property
            # centroid = region.centroid
            x,y,label = self.clustered_frontier[key]
            region = measure.regionprops(self.labeled_frontier_map)[label]
            bbox = region.bbox
            x1,y1,x2,y2 = bbox
            covered_area = (x2-x1)*(y2-y1)
            size = region.area
            no_of_points = len(region.coords)
            density = covered_area/size
            density_cost[key] = density 
        return density_cost

    def get_clusters_with_robot_orientation(self, clusters_keys):
        ''' this function filters the frontier clusters with robot orientation'''
        orientation_cost = {}
        for key in clusters_keys:
            # get centroid of each frontier using regionprops property
            # centroid = region.centroid

            x,y,label = self.candidate_frontier[key]
            x_r, y_r,yaw = self.current_pose
            orientation_diff= wrap_angle(math.atan2(y-y_r,x-x_r))
            orientation_cost[key] = orientation_diff

        return orientation_cost
    

    def get_clusters_with_higher_information_gain(self, regions):
        ''' this function filters the frontier clusters with higher information gain'''


        pass

    def get_odom(self, odom):
        # print("get_odom")
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        # print ("current_pose",self.current_pose)
        # TODO: Store current position (x, y, yaw) as a np.array in self.current_pose var.
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
        # print ("current_pose",self.current_pose)
    def publish_clustered_frontiers(self):
            
            self.clustered_marker.markers = []
            myMarker = Marker()
            myMarker.header.stamp = rospy.Time.now()
                
            myMarker.header.frame_id = "odom"
            myMarker.type = myMarker.POINTS
            myMarker.action = myMarker.ADD
            myMarker.id = 0
            myMarker.color=ColorRGBA(0.5, 0.5, 1, 1)
    
            myMarker.scale.x = 0.1
            myMarker.scale.y = 0.1
            myMarker.scale.z = 0.1
            myMarker.pose.orientation = Quaternion()
            myMarker.pose.orientation.x = 0.0
            myMarker.pose.orientation.y = 0.0
            myMarker.pose.orientation.z = 0.0
            myMarker.pose.orientation.w = 1.0
           
    
            for f in self.clustered_frontier:
                
                frontier_world = self.svc.__map_to_position__(f[0:2]) 
                myPoint = Point()
                myPoint.x = frontier_world[0]
                myPoint.y = frontier_world[1]
                myPoint.z = 0.1
    
                myMarker.points.append(myPoint)
            

            self.clustered_marker.markers.append(myMarker)
            self.cluster_pub.publish(self.clustered_marker)

    def publish_frontiers(self):

        self.frontier_marker.markers = []
        myMarker = Marker()
        myMarker.header.stamp = rospy.Time.now()
            
        myMarker.header.frame_id = "odom"
        myMarker.type = myMarker.POINTS
        myMarker.action = myMarker.ADD
     
        myMarker.color=ColorRGBA(0.224, 1, 0, 1)

        myMarker.scale.x = 0.03
        myMarker.scale.y = 0.03
        myMarker.scale.z = 0.03
        myMarker.id = 0

        for f in self.frontier_list:
                 
            frontier_world = self.svc.__map_to_position__(f) 
            myPoint = Point()
            myPoint.x = frontier_world[0]
            myPoint.y = frontier_world[1]
            myPoint.z = 0

            myMarker.points.append(myPoint)
            
        self.frontier_marker.markers.append(myMarker)
        self.frontier_pub.publish(self.frontier_marker)
    
    def dilate_obstacle(self):
        ''' this function dilates the obstacle to consider the robot size'''

        expanasion_size = int(self.robot_radius/self.svc.resolution)
        binary_map = (self.grid_map>50).astype(np.uint8)
        kernel = np.ones((expanasion_size*2+1,expanasion_size*2+1),np.uint8)
        dilated_map = cv2.dilate(binary_map.astype(np.uint8), kernel, iterations=1)
        # Update the grid map with the expanded obstacles
        dilated_obstacles = (dilated_map * 100).astype(np.int32)
        # Update the grid map with the expanded obstacles
        expanded_obstacles = np.logical_and(dilated_obstacles == 100, self.grid_map == 0)
        self.grid_map[expanded_obstacles] = 100

    def save_image(self):
      
        img_norm = cv2.normalize(self.frontier_map, None, 0, 255, cv2.NORM_MINMAX)
        # Convert the image to uint8 type
        label_img = img_norm.astype(np.uint8)
        # Apply a color map to the image
        color_img = cv2.applyColorMap(label_img, cv2.COLORMAP_HOT)
        cv2.imwrite("frontier_map.png",color_img)

        grid = copy.deepcopy(self.grid_map)
        occu = copy.deepcopy(grid)
        grid[np.where(grid==50)]=127 # Unknown space
        grid[np.where(grid==0)]=255 # free space
        grid[np.where(grid==100)]=0 # occupied space
        cv2.imwrite("testing.png",grid)
        grid = cv2.imread("testing.png")
       
      
    
if __name__ == '__main__':
    rospy.init_node('frontier_exploration')
    frontier_exploration = FrontierExploration()
    rospy.spin()