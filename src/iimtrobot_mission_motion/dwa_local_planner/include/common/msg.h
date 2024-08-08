#ifndef MSG_H
#define MSG_H
#include <angles/angles.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/DWAPlannerConfig.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <cmath>
#endif