#ifndef BEHAVIOR_BASE_H
#define BEHAVIOR_BASE_H

#include <atomic>

#include "common/common.h"

class BehaviorBase {
 public:
  BehaviorBase(costmap_2d::Costmap2DROS *costmap_ros){};
  BehaviorBase() = default;
  ~BehaviorBase() = default;

  virtual void UpdateRobotVel(geometry_msgs::PoseStamped &robot_vel) { robot_vel_ = robot_vel; }
  virtual void UpdateRobotPose(geometry_msgs::PoseStamped &current_pose) { current_pose_ = current_pose; }
  virtual void ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) {}
  virtual bool IsGoalReached() { return isGoalReached_; }
  virtual void SetGoalReached(bool is_goal_reached) { isGoalReached_ = is_goal_reached; }
  bool IsGoalLatched(const tf2_ros::Buffer &tf, const PoseStampedVector &global_plan,
                     const std::string &global_frame, const geometry_msgs::PoseStamped &global_pose,
                     double xy_goal_tolerance);
  bool GetGoalPose(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
                   const std::string &global_frame, geometry_msgs::PoseStamped &goal_pose);
  inline void setZeroSpeed(geometry_msgs::Twist &cmd_vel) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  void resetLatching() { xy_tolerance_latch_ = false; }

 private:
  bool xy_tolerance_latch_ = false;

 protected:
  geometry_msgs::PoseStamped robot_vel_;
  geometry_msgs::PoseStamped current_pose_;
  //   geometry_msgs::PoseStamped current_posestamped_;
  bool isGoalReached_;
  double dis_to_end_;
};
// BehaviorManager类，使用单例模式
#endif