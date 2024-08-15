#ifndef BEHAVIOR_FREE_NAV_H
#define BEHAVIOR_FREE_NAV_H
#include <common/common.h>
#include <common/msg.h>
#include <common/path_manager.h>
#include <common/visualizer_util.h>
#include <common_base/fsm_base.h>
#include <controller/dwa_controller.h>

#include "behavior/behavior_base.h"
using namespace dwa_local_planner;
// using namespace math_util;
class BehaviorFreeNav : public BehaviorBase {
 public:
  BehaviorFreeNav(tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros,
                  costmap_2d::Costmap2D *costmap_2d, std::shared_ptr<DWAPlanner> &dp,
                  std::shared_ptr<VisualizerUtil> &visualizer_util);
  ~BehaviorFreeNav() = default;
  void ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) override;
  void SetPlan(const PoseStampedVector &global_plan);

 private:
  FreeNavEvent ExecIDLE(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                        const PoseStampedVector &transformed_plan);
  FreeNavEvent ExecMoveToFinalGoal(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                   const PoseStampedVector &transformed_plan);
  FreeNavEvent ExecRotationInPlace(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                   const PoseStampedVector &transformed_plan);
  FreeNavEvent ExecRotationAtGoalPos(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                     const PoseStampedVector &transformed_plan);
  FreeNavEvent ExecNormalRunning(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                 PoseStampedVector &transformed_plan);
  FreeNavEvent ExecRequestNewPlan(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                  const PoseStampedVector &transformed_plan);

  bool ComputeVelWithDWA(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist &cmd_vel);
  bool GetGoalPose(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
                   const std::string &global_frame, geometry_msgs::PoseStamped &goal_pose);

  bool IsGoalLatched(const PoseStampedVector &global_plan, const std::string &global_frame,
                     const geometry_msgs::PoseStamped &global_pose, double xy_goal_tolerance);

 private:
  tf2_ros::Buffer *tf_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_2d_;
  std::shared_ptr<DWAPlanner> &dp_;

  ros::Time transformed_plan_empty_time_;
  std::string global_frame_id_;
  std::string base_frame_id_;

  FsmMachine<FreeNavState, FreeNavEvent> state_machine_;
  void CreateFsmState();
  std::map<FreeNavState,
           std::function<FreeNavEvent(geometry_msgs::Twist &, NavStatusInfo &, PoseStampedVector &)>>
      state_map_fun_;

  std::unique_ptr<PathManager> path_manager_;
  std::shared_ptr<VisualizerUtil> &visualizer_util_;

  double xy_goal_tolerance_ = 0.35;
  double xy_goal_precise_tolerance_ = 0.05;
  bool xy_tolerance_latch_ = false;
};
#endif
