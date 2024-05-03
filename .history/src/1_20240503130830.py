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

 self.Q = np.array([[ self.left_wheel_sensor_noise**2, 0],[0 ,self.right_wheel_sensor_noise**2]])


        # icp registration uncertainty 
        self.icpNoise_x = 0.05 + 0.05
        self.icpNoise_y = 0.05  + 0.05
        self.icpNoise_theta = 0.05
        self.R = np.array([[ self.icpNoise_x**2, 0, 0],[0, self.icpNoise_y**2, 0],[0, 0, self.icpNoise_theta**2]]).reshape(3,3)

        # IMU uncertainity
        self.imu_noise= np.array([0.0007**2]).reshape(1,1)

        # initialize the map 
        self.Xk = np.array([self.x, self.y, self.th]).reshape(-1,1)

        # map covariance (SLAM)
        self.P_map = self.P # initialize it with robot uncertainty



        # Initialize linear velocity and angular velocity of the robot
        self.v = 0.0
        self.w = 0.0 
        self.tf_br = TransformBroadcaster()
        # odom publisher
        self.odom_pub = rospy.Publisher("/kobuki/odom", Odometry, queue_size=10)


        self.last_time = rospy.Time.now()

# initialize the map 
        self.Xk = np.array([self.x, self.y, self.th]).reshape(-1,1)

        # map covariance (SLAM)
        self.P_map = self.P # initialize it with robot uncertainty



        # Initialize linear velocity and angular velocity of the robot
        self.v = 0.0
        self.w = 0.0 
        self.tf_br = TransformBroadcaster()
        # odom publisher
        self.odom_pub = rospy.Publisher("/kobuki/odom", Odometry, queue_size=10)
        

        self.last_time = rospy.Time.now()

        # joint state subscriber
        # self.js_sub = rospy.Subscriber("/kobuki/joint_states", JointState, self.joint_state_callback, buff_size = 15)
        self.js_sub = rospy.Subscriber("/turtlebot/joint_states", JointState, self.joint_state_callback, buff_size = 15)

        # Lidar subscriber
        # rospy.Subscriber('/kobuki/sensors/rplidar', LaserScan, self.lidar_callback, buff_size = 5)
        rospy.Subscriber('/turtlebot/kobuki/sensors/rplidar', LaserScan, self.lidar_callback, buff_size = 20)

        # imu subscriber
        # rospy.Subscriber('/kobuki/sensors/imu', Imu, self.imu_callback)
        rospy.Subscriber('/turtlebot/kobuki/sensors/imu_data', Imu, self.imu_callback)

        # Ground Truth Subscriber

C_pub = rospy.Publisher('deadReck_pointCloud', PointCloud2, queue_size=10)

        # # Initialize Marker Publisher of robot poses
        # # self.pose_pub = rospy.Publisher('poseMarkers', Marker, queue_size=10)
        # self.pose_pub = rospy.Publisher('poseMarkers', MarkerArray, queue_size=10)


        

        # # Initialize Path Publishers
        # self.slamTraject_pub = rospy.Publisher('slam_Trajectory', Path, queue_size=10) 
        # self.Grnd_Truth_path_pub = rospy.Publisher('Grnd_Truth_PATH', Path, queue_size=10)
        # self.DeadReck_path_pub = rospy.Publisher('DeadReck_Path', Path, queue_size=10) 
        

        
        # # Ground Truth Path: Red Color is used to display this path, Please set the color in Rviz

