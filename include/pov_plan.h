#ifndef __POV__PLAN_H
#define __POV__PLAN_H

#include <common.h>

class POVPlan
{
private:
	cv::Mat image; // Segmented Navigable Space 
    cv::Mat optic_path_image;
	std::vector<cv::Point> path;
    std::vector<cv::Point> lpath;
    std::vector<cv::Point> rpath;
    
    /* Proximity feature */
    int obs_dist;
    
    float goal_distance;
    
    /* Assigned weights to the object functions. */
    double w1, w2;

    bool VISUALIZATION;

public:
    POVPlan();
	~POVPlan();

    // features (s1, s3)
	cv::Point lp_node, rp_node;

    // Getters
    cv::Mat get_image();
    cv::Mat get_optic_path_image();
    std::vector<cv::Point> get_path();
    std::vector<cv::Point> get_lpath();
    std::vector<cv::Point> get_rpath();
    
    int get_obs_dist();
    
    float get_goal_distance();

    double get_w1();
    double get_w2();

	// Setters
    void set_image(cv::Mat image); 
    void set_optic_path_image(cv::Mat optic_path_image);
    void set_path(std::vector<cv::Point> path);
    void set_lpath(std::vector<cv::Point> lpath);
    void set_rpath(std::vector<cv::Point> rpath);
    
    void set_w1(double w1);
    void set_w2(double w2);
    
    void set_visualization(bool);
    
    // Objective functions
    // Objective 1 : Deviation from goal direction (theta)
    float objective1(cv::Point p, cv::Point global_optic_goal, cv::Point start);
    // Objective 2 : Distance travelled towards the goal (pixels)
    float objective2(cv::Point p, cv::Point global_optic_goal);

    // POG
	cv::Point get_global_optic_goal(geometry_msgs::PoseStamped goal, geometry_msgs::PoseStamped robot_state);
    // HOG
    cv::Point get_local_optic_goal(cv::Point global_optic_goal, cv::Point start);
    // Image based path planning
    cv::Point find_optic_path(int n, cv::Point global_optic_goal, cv::Point local_optic_goal);
};

#endif // __POV__PLAN_H
