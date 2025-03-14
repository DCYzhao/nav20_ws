#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H
#include <common/common.h>
#include <common/math_util.h>
#include <common/msg.h>
#include <tf2_ros/transform_listener.h>
class PathManager {
 public:
  PathManager(tf2_ros::Buffer* tf, costmap_2d::Costmap2D* costmap_2d);
  ~PathManager() = default;

  bool PrunePlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& origin_robot_pose,
                 PoseStampedVector& transformed_plan, double check_obs_dis);
  void SetPlan(const PoseStampedVector& new_path);
  bool GetTransformedPlan(const geometry_msgs::PoseStamped& origin_robot_pose,
                          PoseStampedVector& transformed_plan);
  void CalFixPathLength();

 private:
  PoseStampedVector origin_path_;
  double fix_total_length_;
  std::string local_frame_;
  // The nearest index of global plan to the robot
  size_t nearest_index_ = 0;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_2d_;
  tf2_ros::Buffer* tf_;
  double look_ahead_dist_ = std::numeric_limits<double>::max();
};

#endif