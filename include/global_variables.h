#ifndef __GLOBAL_VARIABLE_H
#define __GLOBAL_VARIABLE_H

#include <common.h>

/* Goal in World Frame */
geometry_msgs::PoseStamped goal;

/* Robot State in World Frame */
geometry_msgs::PoseStamped robot_state;
double robot_heading;

/* Camera Info */
sensor_msgs::CameraInfo cam_info;

#endif // __GLOBAL_VARIABLE_H