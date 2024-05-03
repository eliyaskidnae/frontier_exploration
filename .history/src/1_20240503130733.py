self.scan_num = 10 # number of scans to store
        self.invalid_scans=[]
        self.update_called = False
        self.confidence_level = 0.99

        # initial pose of the robot
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.X= np.array([[self.x], [self.y], [self.th]]).reshape(3,1)
        self.Xdeadreck = np.array([[self.x], [self.y], [self.th]]).reshape(3,1)
        self.Xdeadreck_store = np.array([[self.x], [self.y], [self.th]]).reshape(3,1)
        self.XGroundTruth = np.array([[self.x], [self.y], [self.th]]).reshape(3,1)

        # initial robot pose uncertainty
        self.p_x=0.01
        self.p_y=0.01
        self.p_th=0.01
        self.P= np.array ([ [self.p_x**2, 0.0, 0.0], [0.0, self.p_y**2, 0.0], [0.0, 0.0, self.p_th**2]])

        # Initialize left and right wheel sensors
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.left_wheel_received = False
        self.right_wheel_received = False

        # wheel sensor uncertainty 
        self.left_wheel_sensor_noise = 0.2 + 0.2

