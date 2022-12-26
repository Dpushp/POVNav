#include <pov_nav.h>
#include <global_variables.h>

POVNav::POVNav(ros::NodeHandle nh_)
:it_(nh_)
{
	// Read the topics from the launch file params
    std::string goal_from_rviz_topic;
    std::string odom_topic;
    std::string navigability_map_topic;
	std::string rgb_image_topic;
    std::string frame_id;
    std::string pov_plan_image;
    
    if (nh_.getParam("/sim_pov_nav/goal_from_rviz_topic", goal_from_rviz_topic) &&
        nh_.getParam("/sim_pov_nav/odom_topic", odom_topic) &&
        nh_.getParam("/sim_pov_nav/navigability_map_topic", navigability_map_topic) &&
		nh_.getParam("/sim_pov_nav/rgb_image_topic", rgb_image_topic) &&
        nh_.getParam("/sim_pov_nav/frame_id", frame_id) &&
        nh_.getParam("/sim_pov_nav/pov_plan_image", pov_plan_image))
    {
        //this->frame_id = frame_id;
        //this->is_realsense = is_realsense;
        ROS_INFO("Received all params!");
    }
    else
    {
        ROS_ERROR("Failed to get param!");
    }

    // Subscribe goal from rviz
    goal_sub = nh_.subscribe(goal_from_rviz_topic, 1, &POVNav::goalCallback, this);
    
    // Subscribe robot's odom
    odom_sub = nh_.subscribe(odom_topic, 1, &POVNav::odomCallback, this);
    // Subscribe to the segmented image
    image_sub_ = it_.subscribe(navigability_map_topic, 1, &POVNav::imageCallback, this);
	// Subscribe to the rgb image
    rgb_image_sub_ = it_.subscribe(rgb_image_topic, 1, &POVNav::RGBimageCallback, this);
    
	// Optic Path image publisher
    optic_path_pub_ = it_.advertise(pov_plan_image, 1);

	// Control publisher
	ctrlVelYawPub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	// Timer
	t_1 = ros::Time::now().nsec;
	start_time = ros::Time::now().nsec;

	// Smooth control params
	SMOOTH_CTRL = true;
	b_previous_ctrl_v = 0.0;
    b_previous_ctrl_omega = 0.0;
    b_MAX_CTRL_RATE = 1;
}

POVNav::~POVNav()
{
}

/* Callback Functions */
/* Goal Subscriber : Rviz */
void POVNav::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg){
	goal = *msg;
	tf::Quaternion q(
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z,
		msg->pose.orientation.w
	);

	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	if (DEBUG)
	{
		std::cout << "Goal_position: \n" << goal.pose.position << std::endl;
		std::cout << "Goal_orientation: " << RAD2DEG(yaw) << std::endl;
	}
}

/* Odom Subscriber */
void POVNav::odomCallback(const nav_msgs::OdometryConstPtr& msg){
	robot_state.header = msg->header;
	robot_state.pose = msg->pose.pose;
	
	tf::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w
	);

	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	robot_heading = yaw;
	
	if (false)
	{
		std::cout << "Robot State : \n" << robot_state << std::endl;
	}
}

/* RGB Image subscriber */
void POVNav::RGBimageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try{
		// Read rgb images from robot simulator
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (DEBUG)
	{
		std::cout << "Received image." << std::endl;
	}

    /*! Copy image. */
    rgb_image = cv_ptr->image.clone();
}

/* Visual Horizon */
cv::Mat visualHorizon(cv::Mat Ir){
	/*
		Implement the logic to get the visual horizon.
		1. Define the navigable and non-navigable classes.
		2. See the visual horizon sub-section in the POVNav paper.
			for all pixels in each colunm
				implement visual horizon eq. 
	*/
	return Ir;
}

/* Segmented Image subscriber */
void POVNav::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try{
		// Read depth images from Turtlebot3 gazebo
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (DEBUG)
	{
		std::cout << "Received image." << std::endl;
	}

	/* Set optic path image for visualization. */
	optic_path.set_optic_path_image(rgb_image);

    /* POVNav : steps */
    /*! 0 :: Get Ir : Copy the binary segmented image. */
    cv::Mat Ir = cv_ptr->image.clone();

	/*! 1 :: Visual Horizon : Eleminate unreachable navigable segments. */
    Ir = visualHorizon(Ir);

    /*! 2 :: from pov planner - Optic Path Object */
    optic_path.set_image(Ir);
	optic_path.set_visualization(true);
	
	/*! 3 :: POG - Get sense of direction */
    cv::Point global_optic_goal = optic_path.get_global_optic_goal(goal, robot_state);
    
    /*! 4 :: HOG - Local sense of direction */ 
		// Start point
	cv::Point start;
	start.x = Ir.cols/2;
	start.y = Ir.rows;
	
	// Update the weights 
	optic_path.set_w1(w1);
	optic_path.set_w2(w2);
    cv::Point local_optic_goal = optic_path.get_local_optic_goal(global_optic_goal, start);
    
    /*! 5 :: Find Optimized Optic Path & get the first node as guider node */
    int n = 5; // Number of nodes in beam element path
    cv::Point guider_node = optic_path.find_optic_path(n, global_optic_goal, local_optic_goal);

	/*! 6 :: Control - Get the control velocity and yaw */
	// Get feature 
	double e_alignment_t = atan2(start.x - guider_node.x, start.y - guider_node.y);
	double e_proximity_t = optic_path.get_obs_dist() - 50; // 50 is the threshold
	double distance_to_goal = optic_path.get_goal_distance();

	// Control
	// Partitioned Visual Servoin
	double proximity_feature = e_proximity_t;
	double alignment_feature = e_alignment_t;

	// partitioned_visual_servoing(e_alignment_t, e_proximity_t, distance_to_goal);
	/*! PID Implementation for Visual Servoing.
	 *  Function Signature:
	 *  PID_visual_servoing::PID_visual_servoing(double _alpha, double Kp, double Ki, double Kd) 
	 *  alpha : low pass filter constant
	 */
	double alpha = 0.8, alpha1 = 0.8, alpha2 = 0.8;
	double Ki = 0.0;
	double Ki1 = 0.0;
	double Ki2 = 0.0;
	
	// V_max control
	ibvs.set_alpha(alpha);
	ibvs.set_Kp(Kp);
	ibvs.set_Kd(Kd);
	ibvs.set_Ki(Ki);

	// proximity control
	ibvs_v.set_alpha(alpha1);
	ibvs_v.set_Kp(Kp1);
	ibvs_v.set_Kd(Kd1);
	ibvs_v.set_Ki(Ki1);

	// alignment control
	ibvs_w.set_alpha(alpha2);	
	ibvs_w.set_Kp(Kp2);
	ibvs_w.set_Kd(Kd2);
	ibvs_w.set_Ki(Ki2);
	
	double dt = ros::Time::now().toSec() - t_1;
	t_1 = ros::Time::now().toSec();
	
	double v_max = ibvs.getControl(distance_to_goal, dt);
	
	double v_proxy = ibvs_v.getControl(proximity_feature, dt);
	double w = ibvs_w.getControl(alignment_feature, dt);
	
	double v = v_max;
	
	// switch linear control velocity
	if (proximity_feature < 0)
	{
		// Prevent the robot from moving forward
		v = v_proxy;
		std::cout << "Obstacle detected!" << std::endl;
	}
	else if (proximity_feature < 20)
	{
		// Slow down the robot
		v = v_proxy;
		std::cout << "Slowing down! Velocity : " << v << std::endl; 2*sin(robot_heading + alignment_feature);
	}
	else
	{
		// Move forward with maximum velocity
		v = v_max;
		std::cout << "Moving with maximum velocity! Velocity : " << v << std::endl;
	}
	
	// Publish the control velocity
	double goal_tolerance = 0.5;
	if (optic_path.get_goal_distance() < goal_tolerance)
	{
		// Stop the robot
		v = 0;
		w = 0;
		std::cout << "Goal reached!" << std::endl;
	}
	publish(v, w);

	/* Publish optic_path_image */
    std_msgs::Header header;
    header.frame_id = "odom";
    sensor_msgs::ImagePtr optic_path_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", optic_path.get_optic_path_image()).toImageMsg();
    optic_path_pub_.publish(optic_path_msg);
	
}

// Control publisher
void POVNav::publish(float ctrl_v, float ctrl_omega){
	if(SMOOTH_CTRL){
		/*! Find rate of change of ctrl values. */
		float ctrl_er_v = b_previous_ctrl_v - ctrl_v;

		/*! Limit ctrl values. */
		float speed_limit_v  = 0.5;
		float speed_limit_omega = 3.14;
		if(std::abs(ctrl_v) > speed_limit_v)
			ctrl_v = speed_limit_v * (std::abs(ctrl_v)/ctrl_v + 0.000000000001);
		if(std::abs(ctrl_omega) > speed_limit_omega)
			ctrl_omega = speed_limit_omega * (std::abs(ctrl_omega)/ctrl_omega + 0.000000000001);
	}
		
	if(true){
		/*! INFO ctrl command.*/
		ROS_INFO("ctrl_vf : %f", ctrl_v);
		ROS_INFO("ctrl_wf : %f", ctrl_omega);
	}
	
	/*! Populate the sensor_msgs::Joy with the current value. */
	geometry_msgs::Twist control_v_omega;
	control_v_omega.linear.x = ctrl_v;
	control_v_omega.angular.z = ctrl_omega;
	
	/*! Publish to the Velocity Yaw rate topic. */
	ctrlVelYawPub.publish(control_v_omega);

	if(SMOOTH_CTRL){
		/*! Update previous_ctrl. */
		b_previous_ctrl_v = ctrl_v;
		b_previous_ctrl_omega = ctrl_omega;
	}
}

// Update configuration
void POVNav::updateConfig(double	_Kp, double _Kd, 
						  double _Kp1, double _Kd1, 
						  double _Kp2, double _Kd2,
						  double _w1, double _w2){
	// Dynamic reconfigure parameters
    Kp = _Kp, Kd = _Kd;
    Kp1 = _Kp1, Kd1 = _Kd1;
    Kp2 = _Kp2, Kd2 = _Kd2;
	w1 = _w1, w2 = _w2;
	// ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f", 
    //         Kp, Kp1, Kd2, Kd, Kd1, Kd2, w1, w2);
}