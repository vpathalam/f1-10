Module A:
	Automatic Emergency Braking
	-> Problem: Prevent the car from crashing while trying new Algorithms
	-> Understand: Real-life implementations, sensors, failure models
	-> Implement: Time-to-collision based braking

	F1Tenth 2D Simulator built in ROS, C++

	Pose Representation & Transforms
	-> Problem: Sensors in different reference frames
	-> Understand: Rigid body transforms
	-> Implement: Pose transformations in ROS

	Electronic Speed Control
	-> Problem: Actuate vehicle via commands in physics units
	-> Understand: PID, motor control, etc.
	-> Implement: Motor controller tuning and related parameters

Module B: 
	Wall-following
	-> Problem: How can we drive the car around the track rather than prevent crashes
	-> Understand: Basics of PID, how to compute error, failure modes
	-> Implement: Wall following in simulation

	Obstacle Avoidance: Follow the Gap
	-> Reactive Navigation: Use immediate sensor information to decide driving command
	-> Planning for Obstacle Avoidance: Use LiDAR for obstacle avoidance on both static and dynamic obstacles

Module C:
	Localization: Scan Matching
	-> One of the most fundamental localization algorithms
	-> Scan Matching, Iterative Closest Point Algorithm
	-> Find corresponding scan points between scan frames and match them to find the transformation between frames

	SLAM: Using Cartographer
	-> Problem: How to use state of the art tools for map building
	-> Understand: Cartographer paper and how it relates to scan matching
	-> Implement: Build maps with Cartographer of race track on the car

	Localization: Particle Filter & Sensor Fusion
	-> Given a map of the world and multiple sensor observations, what is the pose of my robot?
	-> Given a map, find the position of the Ego Vehicle (myself) with reference to the world (frame)
	-> Algorithms: Particle Filter (version of Bayesian Filter), Use bayesian Filtering to estimate state of the vehicle in the world frame represented by a map

	Pure Pursuit
	-> Problem: How to track reference trajectory?
	-> Understand: Closed form geometric approach and alternatives
	-> Implement: Pure pursuit waypoint tracker

Module D:
	Motion Planning
	1. Search-based motion planning
	2. Cellular methods
	3. Probabilistic methods
	4. RRT

	Module Predictive Control
	-> Problem: Create dynamically feasible trajectories for overtaking
	-> Understand: Trajectory optimization & sampling based MPC
	-> Implement: 

	Raceline Optimization
	-> Problem: What is the fastest way to get around the track
	-> Understand: Convex optimization approach, modern evolutionary methods
	-> Implement: Raceline optimization as a project

Module E:
	Vision: Detection & Pose Estimation
		1. Geometrical Vision: single view geometry can be used to estimate pose of the april tags. Given the pose of the obstacle, a trajectory can be planned around it to avoid the obstacle vehicle during head to head racing.
		2. YOLO: can be used as another way to detect and estimate the future pose of the vehicle. Will have to train the model with the data input of poses of the vehicle and the images.
		3. Nodelets: A ROS programming element which helps in eliminating overhead of serializing and sending data heavy messages over the ROS topics.
		Summary: Algorithms include Geometrical Vision ( Single View geometry ), Deep Learning ( YOLO : You Only Look Once ), Weighted Pose Prediction
