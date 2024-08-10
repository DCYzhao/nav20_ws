#ifndef BEHAVIOR_FIX_FOLLOW_H
#define BEHAVIOR_FIX_FOLLOW_H
#include <common/common.h>
#include <common/msg.h>
#include <common_base/fsm_base.h>

#include "behavior/behavior_base.h"
class BehaviorFixNav : public BehaviorBase {
 public:
  BehaviorFixNav(tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
  ~BehaviorFixNav() = default;
  void ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) override;
  void SetPlan(const PoseStampedVector &global_plan);
  void UpdateRobotVel(geometry_msgs::PoseStamped &robot_vel) override { robot_vel_ = robot_vel; }
  void UpdateRobotPose(geometry_msgs::PoseStamped &current_pose) override { current_pose_ = current_pose; }

 private:
  FreeNavEvent ExecIDLE(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                        const PoseStampedVector &transformed_plan);
  FreeNavEvent ExecRotationInPlace(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                   const PoseStampedVector &transformed_plan);
  FreeNavEvent ExecNormalRunning(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                 PoseStampedVector &transformed_plan);
  FreeNavEvent ExecRequestNewPlan(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                  const PoseStampedVector &transformed_plan);

 private:
  tf2_ros::Buffer *tf_;
  costmap_2d::Costmap2DROS *costmap_ros_;

  FsmMachine<FreeNavState, FreeNavEvent> state_machine_;
  void CreateFsmState();
  std::map<FreeNavState,
           std::function<FreeNavEvent(geometry_msgs::Twist &, NavStatusInfo &, PoseStampedVector &)>>
      state_map_fun_;
};
#endif