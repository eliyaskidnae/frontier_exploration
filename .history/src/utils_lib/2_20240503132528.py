

 J1_X = np.array([1, 0, -self.v*np.sin(self.X[2][0])*dt]) 
            #J2_X = [del fx / del left wheel sensor noise , del fx / del right wheel sensor noise]
            J2_X = np.array([ 0.5*self.left_wheel_radius*dt*np.cos(self.X[2][0]) , 0.5*self.right_wheel_radius*dt*np.cos(self.X[2][0]) ]) 
            # J1_Y = [del fy / del x, del fy / del y, del fy / del theta]
            J1_Y = np.array([0, 1, self.v*np.cos(self.X[2][0])*dt])   
            #J2_Y = [del fy / del left wheel sensor noise , del fy / del right wheel sensor noise]
            J2_Y = np.array([ 0.5*self.left_wheel_radius*dt*np.sin(self.X[2][0]) , 0.5*self.right_wheel_radius*dt*np.sin(self.X[2][0]) ])
            # J1_theta = [del ftheta / del x, del ftheta / del y, del ftheta / del theta]
            J1_theta = np.array([0, 0, 1]) 
            #J2_theta = [del ftheta / del left wheel sensor noise , del ftheta / del right wheel sensor noise]
            J2_theta = np.array([ -1*self.left_wheel_radius*dt / self.wheel_base_distance , self.right_wheel_radius*dt / self.wheel_base_di>



            J1 = np.stack((J1_X, J1_Y, J1_theta), axis=-1).T 
            J2 = np.stack((J2_X, J2_Y, J2_theta), axis=-1).T


            # Covariance compounding
            self.P = J1 @ self.P @ J1.T + J2 @ self.Q @ J2.T

            # update the map covariance because the robot has moved
            self.P_map[-3:,-3:]= self.P # updating the current robot state covariance 

            # updating the cross covariances of the robot with the previous poses, the cross covariance between the previous poses remain u>
            if self.P_map.shape[0] > 3:
                for i in range(int(self.P_map.shape[0] / 3) ):
                    self.P_map[3*i:3*i+3,-3:] = self.P_map[3*i:3*i+3,-3:] @ J1.T

            if self.P_map.shape[1] > 3:
                for i in range(int(self.P_map.shape[0] / 3) ):
                    self.P_map[-3: , 3*i:3*i+3] = J1 @ self.P_map[-3: , 3*i:3*i+3] 



# reading from usbl, Updating step for the kalman filter is done here
    def update(self, data, point_cloud):   

        over_laps = self.check_overlap()
        overlap_scan_found = False

        if len(over_laps) > 0:
            # overlap_scan_found = False
            for i in over_laps:

                print('over_laps and iiii ', over_laps,   i)

                # Observation model h and its Jacobian H and the noise of the sensor
                # h = np.array ([[  -self.Xk[3*i]*np.cos(self.Xk[3*i+2]) - self.Xk[3*i+1] * np.sin(self.Xk[3*i+2]) + self.Xk[-3]* np.cos(se>
                h = np.array ([[  -self.Xk[3*i]*np.cos(self.Xk[3*i+2]) - self.Xk[3*i+1] * np.sin(self.Xk[3*i+2]) + self.Xk[-3]* np.cos(self>
                # print("hhhh", h)

                # temp1 =  del h / del x, del h / del y, del h / del theta ---- > with the respect to the pose of the matching scan which w>
                temp1 = np.array([ [-np.cos(self.Xk[3*i+2][0]), -np.sin(self.Xk[3*i+2][0]), self.Xk[3*i][0] * np.sin(self.Xk[3*i+2][0]) - s>
                temp1 = temp1.reshape(3, 3)
                # temp2 =  del h / del x, del h / del y, del h / del theta ---- > with the respect to the pose from which the new scan was >
                temp2 = np.array([[np.cos(self.Xk[3*i+2][0]), np.sin(self.Xk[3*i+2][0]), 0], [-np.sin(self.Xk[3*i+2][0]), np.cos(self.Xk[3*>



                self.H = np.zeros((3, len(self.Xk))).reshape(3,-1)

  temp1 = temp1.reshape(3, 3)
                # temp2 =  del h / del x, del h / del y, del h / del theta ---- > with the respect to the pose from which the new scan was >
                temp2 = np.array([[np.cos(self.Xk[3*i+2][0]), np.sin(self.Xk[3*i+2][0]), 0], [-np.sin(self.Xk[3*i+2][0]), np.cos(self.Xk[3*>



                self.H = np.zeros((3, len(self.Xk))).reshape(3,-1)

                self.H[:, 3*i:3*i+3] = temp1
                self.H[:, -3:] = temp2
                self.H.reshape(3,-1)

                self.V = np.eye(3)

                self.z = icp_register(data,self.scans[i], h)
                self.z = self.z.reshape(3,1)
                # print("self.zzzz", self.z)

                print("error z-h", np.linalg.norm(self.z[:2]-h[:2]))
                print(" z and h", self.z, h)
                # UPDATING STEP OF KALMAN FILTER
                # if np.linalg.norm(self.z-h) < 0.1 :

                # Compatibility Test

                J_minus = np.array([[-np.cos(self.Xk[3*i+2][0]), -np.sin(self.Xk[3*i+2][0]), self.Xk[3*i][0]*np.sin(self.Xk[3*i+2][0])- sel>
                J1_plus = temp1
                J2_plus = temp2

                inverted_cov = J_minus @ self.P_map[3*i:3*i+3,3*i:3*i+3] @ J_minus.T
                S = J1_plus @ inverted_cov @ J1_plus.T + J2_plus @ self.P_map[-3:,-3:] @ J2_plus.T
