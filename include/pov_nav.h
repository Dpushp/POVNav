#ifndef __POV_NAV_H
#define __POV_NAV_H

#include <pid_IBVS.h>
#include <pov_plan.h>
#include <common.h>

class POVNav
{
private:
    POVPlan optic_path;
    cv::Mat rgb_image;
    PID_IBVS ibvs;
    PID_IBVS ibvs_v;
    PID_IBVS ibvs_w;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber rgb_image_sub_;
    image_transport::Publisher optic_path_pub_;
    ros::Subscriber goal_sub;
    ros::Subscriber odom_sub;

    double t_1; // previous time stamp
    double start_time;

    // Smooth control params
    bool SMOOTH_CTRL = true;
    float b_previous_ctrl_v;
    float b_previous_ctrl_omega;
    float b_MAX_CTRL_RATE;
    ros::Publisher ctrlVelYawPub;

    // Dynamic reconfigure parameters
    double Kp, Kd;
    double Kp1, Kd1;
    double Kp2, Kd2;
    double w1 = 100, w2 = 1; // default values

public:
    POVNav(ros::NodeHandle nh_);
    ~POVNav();

    /* Callback Functions */
    /* Goal Subscriber : Rviz */
    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    /* Odom Subscriber */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    /* Segmented Image subscriber */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    /* RGB Image subscriber */
    void RGBimageCallback(const sensor_msgs::ImageConstPtr& msg);
    /* Dynamic Reconfigure */
    void updateConfig(double kp, double kd, double kp1, double kd1, double kp2, double kd2, double w1, double w2);

    // Publish control
    void publish(float ctrl_v, float ctrl_omega);
};

#endif // __POV_NAV_H