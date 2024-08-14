#ifndef BEHAVIOR_FREE_NAV_H
#define BEHAVIOR_FREE_NAV_H
#include <common/common.h>
#include <common/msg.h>
#include <common/path_manager.h>
#include <common_base/fsm_base.h>
#include <controller/dwa_controller.h>

#include "behavior/behavior_base.h"
using namespace dwa_local_planner;
class BehaviorFreeNav : public BehaviorBase {
 public:
  BehaviorFreeNav(tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros,
                  costmap_2d::Costmap2D *costmap_2d, std::shared_ptr<DWAPlanner> &dp);
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
};
#endif
