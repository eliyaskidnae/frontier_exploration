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





   # Compatibility Test
                
                J_minus = np.array([[-np.cos(self.Xk[3*i+2][0]), -np.sin(self.Xk[3*i+2][0]), self.Xk[3*i][0]*np.sin(self.Xk[3*i+2][0])- sel>
                J1_plus = temp1
                J2_plus = temp2

                inverted_cov = J_minus @ self.P_map[3*i:3*i+3,3*i:3*i+3] @ J_minus.T
                S = J1_plus @ inverted_cov @ J1_plus.T + J2_plus @ self.P_map[-3:,-3:] @ J2_plus.T
                

                mahalanobis_dist = distance.mahalanobis(np.ravel(h), np.ravel(self.z), np.linalg.inv(S + self.R))
                # print("S + self.R", np.linalg.inv(S + self.R))
                # Find the critical value
                critical_value = chi2.ppf(self.confidence_level, 3)
                print("critical_value", critical_value)
                print("mahalanobis_dist", mahalanobis_dist, mahalanobis_dist**2)
                print("(mahalanobis_dist**2) <=critical_value", (mahalanobis_dist**2) <=critical_value)
                # if np.linalg.norm(self.z[:2]-h[:2]) < 0.25 and abs(h[2][0] - self.z[2][0] ) < 0.2:# np.linalg.norm(self.z-h) < 0.1 :
                if (mahalanobis_dist**2) <=critical_value:
                    # computing the kalman gain
                    print("Updating .... ")
                    overlap_scan_found = True
                    K = self.P_map @ self.H.T @ np.linalg.inv( self.H @ self.P_map @ self.H.T + self.V @ self.R @ self.V.T)
                    
                    
                
                    # updating the mean value of the state vector
                    self.Xk = self.Xk + K @ (self.z - h)

                    
                    # updating the covariance  
                    self.P_map = (np.eye(self.P_map.shape[0]) - K @ self.H ) @ self.P_map # @ ( np.eye(self.P_map.shape[0]) - K @ self.H).T 

