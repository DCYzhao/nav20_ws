#ifndef VISUALIZER_UTIL_H
#define VISUALIZER_UTIL_H
#include <base_local_planner/goal_functions.h>
#include <common/common.h>

class VisualizerUtil {
 public:
  VisualizerUtil(ros::NodeHandle& private_nh);
  ~VisualizerUtil();
  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }

  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }
  void publishTransformedPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, transform_plan_pub_);
  }

 private:
  ros::Publisher g_plan_pub_, l_plan_pub_, transform_plan_pub_;
};
#endif