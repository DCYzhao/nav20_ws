#ifndef CONTROLLER_PP_H
#define CONTROLLER_PP_H
#include <common/common.h>
class PPController {
 public:
  PPController();
  ~PPController() = default;
  bool getForwardPose(geometry_msgs::PoseStamped &current_pose, const PoseStampedVector &transformed_plan,
                      geometry_msgs::Pose2D &forwardPose);
  void puresuitFollowPath(geometry_msgs::PoseStamped &current_pose, geometry_msgs::Pose2D &forwardPose,
                          geometry_msgs::Twist &cmd_vel);

 private:
  void getLookAheadPoint(geometry_msgs::PoseStamped &current_pose, const double &lookahead_dist,
                         const PoseStampedVector &transformed_plan, geometry_msgs::Pose2D &forwardPose);
  double getLookAheadDistance(const double &speed);

 private:
  double rotate_to_heading_min_angle_ = 0.95;
  double check_path_curvature_dist_ = 4.0;
  double robot_speed_;
  bool use_velocity_scaled_lookahead_dist_ = true;  // 前向跟踪点是否与速度变动
  double forward_dist_ = 0.3;
  double max_lookahead_dist_ = 0.9;
  double min_lookahead_dist_ = 0.3;
  double lookahead_time_ = 0.9;
};
#endif