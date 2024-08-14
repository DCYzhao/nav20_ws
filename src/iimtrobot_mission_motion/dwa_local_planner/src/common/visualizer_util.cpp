#include "common/visualizer_util.h"

VisualizerUtil::VisualizerUtil(ros::NodeHandle& private_nh) {
  g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
  l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  transform_plan_pub_ = private_nh.advertise<nav_msgs::Path>("transform_plan", 1);
}
