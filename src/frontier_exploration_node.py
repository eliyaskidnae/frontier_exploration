
#!/usr/bin/env python
from spatialmath import Quaternion
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from utils_lib.online_planning import StateValidityChecker
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from skimage import measure
import cv2
import copy

class FrontierExploration:
    def __init__(self):
                # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        self.frontier_map = []
        self.frontier_list = []
        self.frontier_marker = MarkerArray()
        self.frontier_marker.markers = []
        self.candidate_marker = MarkerArray()
        self.candidate_marker.markers = []
        self.svc = StateValidityChecker(  )
        # self.frontier_exploration_pub = rospy.Publisher('/frontier_exploration', String, queue_size=10)
        # self.frontier_exploration_sub = rospy.Subscriber('/frontier_exploration', String, self.frontier_exploration_callback)
        self.frontier_pub = rospy.Publisher('/frontier', MarkerArray, queue_size=2)
        self.candidate_pub = rospy.Publisher('/candidate_frontier', MarkerArray, queue_size=2)
        # SUBSCRIBERS
        self.gridmap = rospy.Subscriber('/projected_map', OccupancyGrid, self.get_gridmap)


    
    def get_gridmap(self, gridmap):
        
        # To avoid map update too often (change value '1' if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 1:            
            self.last_map_time = gridmap.header.stamp
            date = str(rospy.Time.now())
            # print('Map received at: ', grid_map2)
            self.grid_map = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
           
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            resolution = gridmap.info.resolution
           
            [self.width,self.height] = self.grid_map.shape
            self.svc.set(self.grid_map, resolution, origin)
            grid_map2 = self.grid_map.copy()
            
            # intialize frontier map
            self.frontier_map = np.zeros_like(self.grid_map)
            self.frontier_list = []
            self.set_frontier_map()


            
    def set_frontier_map(self):
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


        # print('Frontier map set',frontier_list)
        self.publish_frontiers()
        self.cluster_frontier()

        
    def cluster_frontier(self):

        labelled_frontier_map = measure.label(self.frontier_map ,  background=0)

        self.save_image()
      
        # Call regionprops function to get characteristics of each frontier
        # Example: size, orientation, centroid

        regions = measure.regionprops(labelled_frontier_map)
        self.candidate_frontiers = []  
        grid = copy.deepcopy(self.grid_map)    
        grid[np.where(grid==50)]=127 # Unknown space
        grid[np.where(grid==0)]=255 # free space
        grid[np.where(grid==100)]=0 # occupied space
        cv2.imwrite("testing.png",grid)
        grid = cv2.imread("testing.png")
        i= 0
        for prop in regions:
            
            
            if prop.area > 4.0:
                # get centroid of each frontier using regionprops property
                x = int(prop.centroid[0])
                y = int(prop.centroid[1])
               
                self.candidate_frontiers.append([x,y])

                self.publish_candidate_frontiers()

    def publish_candidate_frontiers(self):
            
            self.candidate_marker.markers = []
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
           
    
            for f in self.candidate_frontiers:
                    
                frontier_world = self.svc.__map_to_position__(f) 
                myPoint = Point()
                myPoint.x = frontier_world[0]
                myPoint.y = frontier_world[1]
                myPoint.z = 0.1
    
                myMarker.points.append(myPoint)
                
            self.candidate_marker.markers.append(myMarker)
            self.candidate_pub.publish(self.candidate_marker)



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