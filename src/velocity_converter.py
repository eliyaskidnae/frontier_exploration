#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class Velocity_Convorter:
    def __init__(self):
        # Create a subscriber to the cmd_vel topic
        self.wheel_radius = 0.035
        self.wheel_base = 0.235 
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        # Create a publisher to the left wheel
       # velocity publisher
        self.vel_pub = rospy.Publisher('/kobuki/kobuki/commands/wheel_velocities', Float64MultiArray , queue_size=10)
        self.vel_pub = rospy.Publisher('/turtlebot/kobuki/commands/wheel_velocities', Float64MultiArray , queue_size=10)

    def callback(self, msg):
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z
        # print("linear and angular ", lin_vel , ang_vel )
        left_linear_vel   = lin_vel  - (ang_vel*self.wheel_base/2)
        right_linear_vel  = lin_vel  +  (ang_vel*self.wheel_base/2)
 
        left_wheel_velocity  = left_linear_vel / self.wheel_radius
        right_wheel_velocity = right_linear_vel / self.wheel_radius
        wheel_vel = Float64MultiArray()
        wheel_vel.data = [left_wheel_velocity, right_wheel_velocity]
        self.vel_pub.publish(wheel_vel)


# Main function
if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('velocity_converter')

    # Create an instance of the DifferentialDrive class
    diff_drive = Velocity_Convorter()
    # Spin
    rospy.spin()




def OverlappingScan(self):

        H0= [] 
        match_pose = self.xk[-6:-3,:][0:2,:]
        print("self.xk",self.xk)
        print("match_pose",match_pose)
        
        for i in range(0 , len(self.xk)-2*self.xb_dim , self.xb_dim ):
            print("New match", self.xk[i:i+3,:][0:2,:])
            distance  = np.linalg.norm(self.xk[i:i+3,:][0:2,:] - match_pose)
            if distance < 1:
                index = int(i/3)
                H0.append(index)
            
        return H0
    
    def hf(self,H):
             
        hf = np.zeros((0,1))
        for i in range(len(H)):           
            if(H[i] != 0):
               hf = np.block([[hf],[self.hfj(self.xk, H[i])]])
        return hf
    

    def hfj(self, xk, j ):
        # h(xk_bar,vk)=(-) Xk) [+] x_J+ vk

        # Get Pose vector from the filter state
        NxBk = self.get_robot_pose(xk) # current pose 
        index = int(j*3)
        
        
        NxBj = self.xk[index:index+3,:].reshape((3,1)) # matched pose

        BjxN = Pose3D.ominus(NxBj)
        hfj  = Pose3D.oplus(NxBk , BjxN)
        
        return hfj


    


 def scan_available(self, msg):
        self.mutex.acquire()
        scan = msg
        self.scan_read = True
    
        # Convert laser scan data to x, y coordinates in robot frame 
        self.scan_cartesian = scan_to_cartesian(scan)

      
        if(len(self.map) == 0):
            self.scan_world = scan_to_world(self.scan_cartesian, self.xk[-3:])
            self.scan.append(self.scan_cartesian)
            self.map.append(self.scan_world)    
            print("scan added")
            self.xk , self.Pk = self.pse.Add_New_Pose(self.xk, self.Pk)

      

        elif(check_distance_bw_scans(self.xk,self.scan_th_distance, self.scan_th_angle)):
            self.scan_world = scan_to_world(self.scan_cartesian, self.xk[-3:])
            self.map.append(self.scan_world)
            self.scan.append(self.scan_cartesian)
            self.xk , self.Pk = self.pse.Add_New_Pose(self.xk, self.Pk)
            Ho = self.pse.OverlappingScan()
            print("scan added")
            print("Ho",Ho)
            print("map",len(self.map) )
            print("xk",self.xk.shape)
            for j in Ho:

                kXj = self.pse.hfj(self.xk, j)

                # print("map",self.map)
                mached_scan = self.scan[j]
                current_scan = self.scan[-1]
                # ICP Registration 
                zk= ICP(mached_scan, current_scan, kXj)
                print("zk",zk)

        if(len(self.map) > 1):
            self.publish_viewpoints()   
            self.publish_full_map()
            
        self.mutex.release() 


   


import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import copy
import time


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])
 


def ICP(MatchedScan , CurrentScan , initial_guess): #, MatchedVp, CurrentVp

    '''Debugging Purposes'''

    x1 = np.copy(MatchedScan[:,0])
    y1 = np.copy(MatchedScan[:,1])
    x2 = np.copy(CurrentScan[:,0])
    y2 = np.copy(CurrentScan[:,1])
    '''==================================='''
   
    # Add a column of zeros to the point clouds to make them 3D
    temp_column = np.zeros(MatchedScan.shape[0])
    source_points = np.hstack((MatchedScan, temp_column.reshape(-1, 1)))
    
    # Add a column of zeros to the point clouds to make them 3D
    temp_column = np.zeros(CurrentScan.shape[0])
    target_points = np.hstack((CurrentScan, temp_column.reshape(-1, 1)))

    # Create Open3D point cloud objects
    source_cloud = o3d.geometry.PointCloud()
    source_cloud.points = o3d.utility.Vector3dVector(source_points)

    target_cloud = o3d.geometry.PointCloud()
    target_cloud.points = o3d.utility.Vector3dVector(target_points)

    color = np.array([1.0, 0.0, 0.0])  # Set the desired color (here, red)
    source_cloud.paint_uniform_color(color)

    initial_guess = initial_guess.flatten()
    #create a 4x4 transformation matrix out of the initial guess
    print("initial_guess",initial_guess.flatten())
    initial_guess = np.asarray([[np.cos(initial_guess[2]), -np.sin(initial_guess[2]), 0, initial_guess[0]],   
                                [np.sin(initial_guess[2]), np.cos(initial_guess[2]), 0, initial_guess[1]],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])


    #convert initial guess to float64
    initial_guess = initial_guess.astype(np.float64)

    # Perform registration
    reg_p2p = o3d.pipelines.registration.registration_icp(
               target_cloud, source_cloud, 3, initial_guess,
               o3d.pipelines.registration.TransformationEstimationPointToPoint(),
               o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000))
    
    transformation = reg_p2p.transformation
    # print("Transformation is:")
    translation = transformation[0:2, 3]
    theta = np.arctan2(transformation[1, 0], transformation[0, 0])
    print("Translation is:", translation , "Rotation is:", theta)
    '''Debugging purposes'''
    # draw_registration_result(source_cloud, target_cloud, transformation)
    
    aligned_pcd1 = source_cloud.transform(transformation)
    p3 = np.asarray(aligned_pcd1.points)

    # Visualize the aligned point clouds
    # o3d.visualization.draw_geometries([aligned_pcd1, target_cloud])

    x3 = p3[:, 0]
    y3 = p3[:, 1]


    fig = plt.figure()
    ax2 = fig.add_subplot()
    ax2.scatter(x1, y1, c='green', s=1)
    ax2.scatter(x2, y2, c='blue', s=1)
    ax2.scatter(x3, y3, c='red', s=1)
 
    ax2.set_title("scan matching using ICP")
    plt.savefig('localization_lab/media/ICP'+str(np.round(time.time(), 2))+'.png')
    plt.close()


    # Extract translation and rotation
    translation = transformation[0:2, 3]
    theta = np.arctan2(transformation[1, 0], transformation[0, 0])
    return [translation[0], translation[1], theta]


