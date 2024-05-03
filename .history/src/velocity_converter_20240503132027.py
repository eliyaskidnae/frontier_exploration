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

