    robot = DifferentialDrive()

    # Register the signal handler
    # signal.signal(signal.SIGINT, robot.signal_handler)

    rospy.spin()
