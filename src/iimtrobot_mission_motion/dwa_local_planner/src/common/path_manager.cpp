#include <common/path_manager.h>

PathManager::PathManager(costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(costmap_ros) {
  local_frame_ = "/odom";
}

bool PathManager::GetTransformedPlan(const geometry_msgs::PoseStamped& origin_robot_pose,
                                     PoseStampedVector& transformed_plan) {
  return PrunePlan(origin_robot_pose, transformed_plan, look_ahead_dist_, false);
}

void PathManager::SetPlan(const PoseStampedVector& new_path) {
  origin_path_.clear();
  origin_path_ = new_path;
  nearest_index_ = 0;
}

void PathManager::CalFixPathLength() {
  fix_total_length_ = 0.0;
  for (int i = 0; i < origin_path_.size() - 1; i++) {
    fix_total_length_ += math_util::CalDistM(origin_path_[i], origin_path_[i + 1]);
  }
}