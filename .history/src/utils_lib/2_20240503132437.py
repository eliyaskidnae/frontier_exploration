

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

