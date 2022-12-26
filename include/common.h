#ifndef __COMMON_H
#define __COMMON_H

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pov_nav/pov_nav_dyparamConfig.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/tf.h>

#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <filesystem>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * ((180.0) / (C_PI)))
 
#define DEBUG true

#endif // __COMMON_H