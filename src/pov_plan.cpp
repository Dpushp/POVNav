#include <pov_plan.h>

POVPlan::POVPlan(){}
POVPlan::~POVPlan(){}

/*  Getters */
cv::Mat POVPlan::get_image(){
    return image;
}

cv::Mat POVPlan::get_optic_path_image(){
    return optic_path_image;
}

std::vector<cv::Point> POVPlan::get_path(){
    return path;
}

std::vector<cv::Point> POVPlan::get_lpath(){
    return lpath;
}

std::vector<cv::Point> POVPlan::get_rpath(){
    return rpath;
}

double POVPlan::get_w1(){
    return w1;
}

double POVPlan::get_w2(){
    return w2;
}

int POVPlan::get_obs_dist(){
    return obs_dist;
}

float POVPlan::get_goal_distance(){
    return goal_distance;
}

/* setters */
void POVPlan::set_image(cv::Mat image){
    this->image = image;
}

void POVPlan::set_optic_path_image(cv::Mat optic_path_image){
    this->optic_path_image = optic_path_image;
}

void POVPlan::set_path(std::vector<cv::Point> path){
    this->path = path;
}

void POVPlan::set_lpath(std::vector<cv::Point> lpath){
    this->lpath = lpath;
}

void POVPlan::set_rpath(std::vector<cv::Point> rpath){
    this->rpath = rpath;
}

void POVPlan::set_w1(double w1){
    this->w1 = w1;
}

void POVPlan::set_w2(double w2){
    this->w2 = w2;
}

void POVPlan::set_visualization(bool VISUALIZATION){
    this->VISUALIZATION = VISUALIZATION;
}

/* Objective functions */
// Objective 1 : Deviation from goal direction (theta)
float POVPlan::objective1(cv::Point p, cv::Point global_optic_goal, cv::Point start){
	float alpha = atan2(start.x - global_optic_goal.x, start.y - global_optic_goal.y);
	float beta = atan2(start.x - p.x, start.y - p.y);
	float f1 = abs(beta-alpha); 	
	return f1;
}


// Objective 2 : Distance travelled towards the goal (pixels)
//			     Maximization problem converted into minimization by changing the objective function as -
//				 || global_optic_goal - p ||.
float POVPlan::objective2(cv::Point p, cv::Point global_optic_goal){
	// Ideally : sqrt(pow((global_optic_goal.x - p.x), 2) + pow((global_optic_goal.y - p.y), 2));
	// Here, only y coordinate matters as x represents objective 1. 
	float f2 = sqrt(pow((global_optic_goal.y - p.y), 2)); 
	return f2;	
}


/* Find optic goal */
// POG - goal direction on the image boundary
cv::Point POVPlan::get_global_optic_goal(geometry_msgs::PoseStamped goal, geometry_msgs::PoseStamped robot_state)
{
	/* Get the robot direction in the World Frame */
	float robot_orientation;
	tf::Quaternion q(
		robot_state.pose.orientation.x,
		robot_state.pose.orientation.y,
		robot_state.pose.orientation.z,
		robot_state.pose.orientation.w
	);

	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	robot_orientation = yaw;

	/* Get the goal vector in world frame */
	cv::Mat goal_w = (cv::Mat_<float>(3, 1) <<  goal.pose.position.x - robot_state.pose.position.x, goal.pose.position.y - robot_state.pose.position.y, goal.pose.position.z - robot_state.pose.position.z);
	// Rotation from Frame F1=World to Frame F2=Robot : alpha -> Yaw, beta->pitch, gamma->roll
	double alpha = -robot_orientation, beta = DEG2RAD(0), gamma = DEG2RAD(0); 
	cv::Mat R_F1_F2 = 
	(cv::Mat_<float>(3, 3) << cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma),
							  sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma),
							 -sin(beta),			cos(beta)*sin(gamma),								   cos(beta)*cos(gamma)  
	);
	cv::Mat goal_R = R_F1_F2*goal_w;
	// Set goal distance
	goal_distance = cv::norm(goal_R, cv::NORM_L2);

	/* Get the goal direction in robot's body frame */
	float goal_direction = atan2(goal_R.at<float>(1,0), goal_R.at<float>(0,0));
	// std::cout << "Theta : " << RAD2DEG(goal_direction) << std::endl;

	/* Get goal direction in Image */
	int l = image.cols;
	int b = image.rows;
	
	// Solution of |y| = l/2, |x| = b and y = mx
	float m_ = tan(goal_direction);
	cv::Point p1(l/(2*m_), l/2), p2(-l/(2*m_), -l/2), p3(b, m_*b), p4(-b, -m_*b);

	// p = min{Pi}
	cv::Point min_p1, min_p2;
	float d1 = cv::norm(p1), d3 = cv::norm(p3);
	if (d1 < d3)
	{
		min_p1 = p1;
		min_p2 = p2;
	}
	else
	{
		min_p1 = p3;
		min_p2 = p4;
	}
	
	// Pick the final point based on the unit vector along goal direction 
	cv::Point p;
	int n_ = goal_direction/abs(goal_direction);
	int n1 = min_p1.y/abs(min_p1.y);
	if (n_*n1 > 0)
	{
		p = min_p1;
	}
	else
	{
		p = min_p2;
	}
	
	// Apply Heaviside step function
	p.x = std::max(0, p.x);
	cv::Point global_optic_goal;
	global_optic_goal.x = l/2 - p.y;
	global_optic_goal.y = b - p.x;
	return global_optic_goal; 
}

// HOG - Pareto-optimal sub goal in the image
cv::Point POVPlan::get_local_optic_goal(cv::Point global_optic_goal, cv::Point start){
	// Robot Footprint 
	int robot_footprint = 20; // unit = pixel 
	cv::Point optic_goal;
	float min_distance = std::numeric_limits<float>::max();
	int _d = std::numeric_limits<int>::max();
	
	// Rewrite : By iterating over H_t 
	for(int x = 1; x < image.cols - 1; ++x){
		for(int y = 1; y < image.rows - 1; ++y){
			if(image.at<bool>(y, x) > 0){
				if (image.rows - y < _d)
				{
					_d = image.rows - y;
				}
				break;	
			}
		}
	}
	obs_dist = _d;

	cv::Point p_star(1,1);
	float p_star_value = std::numeric_limits<float>::max();
	for(int x = 1; x < image.cols - 1; ++x){
		for(int y = 1; y < image.rows - 1; ++y){
			if(image.at<bool>(y, x) > 0 && 
			image.at<bool>(y, x - robot_footprint/2) > 0 && 
			image.at<bool>(y, x + robot_footprint/2) > 0){
				cv::Point p(x, y);
				float f1 = objective1(p, global_optic_goal, start);
				float f2 = objective2(p, global_optic_goal);
				float optimal_value = w1*f1 + w2*f2;

				if(optimal_value < p_star_value){
					p_star_value = optimal_value;
					p_star = p;
				}

			}
		}
	}
	return p_star;
}

/*! Generate Features - Find n-2D Beam optimized path elements and return next point to follow 
	Note: Any path planning algorithm can be used if we can consider the robot's size during planning.
*/
cv::Point POVPlan::find_optic_path(int n, cv::Point global_optic_goal, cv::Point local_optic_goal){
	// Size of image
	int rows = optic_path_image.rows;
	int cols = optic_path_image.cols;

	// Start point
	cv::Point start;
	start.x = cols/2;
	start.y = rows;

	// N 2D finite elements points
	std::vector<cv::Point> pts_fe;
	std::vector<cv::Point> l_path;
    std::vector<cv::Point> r_path;
	
	if (VISUALIZATION)
	{
		// Show POG
		cv::circle(optic_path_image, cv::Point(global_optic_goal.x, global_optic_goal.y), 30, CV_RGB(0, 0, 255), -1);
		// Local optic goal 
		cv::circle(optic_path_image, cv::Point(local_optic_goal.x, local_optic_goal.y), 10, CV_RGB(0, 255, 0), -1);
		// Connect with a line
		cv::line(optic_path_image, cv::Point(local_optic_goal.x, local_optic_goal.y), cv::Point(global_optic_goal.x, global_optic_goal.y), CV_RGB(255, 0, 0), 2);

		// Show start point
		cv::circle(optic_path_image, cv::Point(start.x, start.y), 50, CV_RGB(0, 255, 0), -1);
		// Initial Path : Connect with a line -> Start to optic goal
		//cv::line(optic_path_image, cv::Point(local_optic_goal.x, local_optic_goal.y), cv::Point(start.x, start.y), CV_RGB(255, 0, 0), 2, cv::LINE_8);

	}
	
	// Path Optimization
	// Parameters
	int gamma = 5;
	// optic behabour {Homogeneous factor}
	int alpha = 1, delta = 50; 
	int m = 4.5; // default : 2.66;

	// int sigma = cols/2; // Maximum allowed movement of nodes
	int saftey = 10; // pixels
	// Push the goal 
	pts_fe.push_back(cv::Point(local_optic_goal.x, local_optic_goal.y));
	// std::cout << "Pushed : " << local_optic_goal << std::endl;
	// Initiate reference node
	cv::Point ref_node(local_optic_goal.x, local_optic_goal.y);
	int dy = alpha*gamma;
	// int sum_y = local_optic_goal.y -140 - start.y/2 + dy;
	// std::cout << local_optic_goal.y - start.y/2;
	int sum_y = dy;

	// Path Optimization
	for(int y = local_optic_goal.y; y < start.y - 10; y = y + dy){		
		// std::cout << "Inside for loop \n";
		float theta = atan2(ref_node.x - start.x, ref_node.y - start.y);
		dy = alpha*gamma;
		sum_y = sum_y + dy;
		int dx = dy * tan(theta);
		int x = ref_node.x + dx;
		// Move the node within the boundary
		int i = 1;
		// for(int x_ = x; x_ > x - sigma && x_ < x + sigma; x_ = x + pow(-1, i) * i){
		for(int x_ = x; x_ > 0 && x_ < image.cols; x_ = x + pow(-1, i) * i){	
			// Limit left x and right x
			int left_x_ = (x_- m*sum_y < 0) ? 0 : x_- m*sum_y;
			int right_x_ = (x_+ m*sum_y >= cols) ? cols - 1 : x_+ m*sum_y; 
			cv::Point l(left_x_, y);
			cv::Point c(x_, y);
			cv::Point r(right_x_, y);
			
			// Check left and right
			int offset = 0; //370;
			if(image.at<bool>(y, left_x_) > 0 && 
			image.at<bool>(y, x_) > 0 &&
			image.at<bool>(y, right_x_) > 0){
				// Add to path
				// l.x = c.x - (m*sum_y + offset) + saftey;
				l_path.push_back(l);
				// Push center
				pts_fe.push_back(c);
				// Right
				// r.x = c.x + (m*sum_y + offset) - saftey;
				r_path.push_back(r);
				// Update reference node
				ref_node = c;
				if (VISUALIZATION)
				{
					//cv::circle(optic_path_image, ref_node, 17, CV_RGB(0, 0, 0), -1);
				}
				break;
			}
			else{
				i++;	
			}
		}
	}
	pts_fe.push_back(cv::Point(start.x, start.y));
	
	// Add to path : left and right features
	cv::Point l_f, r_f;
	l_f.x = start.x - m*(rows+10-local_optic_goal.y) + saftey;
	l_f.y = rows;
	l_path.push_back(l_f);
	
	// Right
	r_f.x = start.x + m*(rows+10-local_optic_goal.y) - saftey;
	r_f.y = rows;
	r_path.push_back(r_f);
	

	if (VISUALIZATION)
	{
		// draws the curve using polylines and line width (GREEN)
		int lineWidth = 10;
		//cv::polylines(optic_path_image, l_path, false, cv::Scalar(255, 0, 0), lineWidth/2);
		cv::polylines(optic_path_image, pts_fe, false, cv::Scalar(0, 255, 0), lineWidth);
		//cv::polylines(optic_path_image, r_path, false, cv::Scalar(255, 0, 0), lineWidth/2);
	}
	// Set optic_path
	set_path(pts_fe);
	set_lpath(l_path);
	set_rpath(r_path);

	// Find the first node to guide the robot towards goal
	cv::Point guider_node;
	
	//pts_fe.pop_back(); // Start node
	//std::cout << "After 1st pop : " << *(pts_fe.rbegin()) << std::endl;
	pts_fe.pop_back(); // Start node
	guider_node = *(pts_fe.rbegin());
	lp_node = *(l_path.rbegin());
	rp_node = *(r_path.rbegin());
	// std::cout << "Guider Node : " << guider_node << std::endl;
	if (VISUALIZATION)
	{
		cv::circle(optic_path_image, guider_node, 5, CV_RGB(255, 0, 0), -1);
		cv::line(optic_path_image, start, guider_node, CV_RGB(255, 0, 0), 5);
	}
	return guider_node;
}
