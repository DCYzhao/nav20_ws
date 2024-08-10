#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <common/common_algorithm.h>
#include <common/msg.h>
#include <dwa_local_planner/state_manager.h>
#include <dynamic_reconfigure/server.h>
#include <nav_core/base_local_planner.h>
#include <nav_core/task_mode.h>

#include "behavior/behavior_base.h"
#include "behavior/behavior_charging.h"
#include "behavior/behavior_fix_follow.h"
#include "behavior/behavior_free_nav.h"
#include "behavior/behavior_idle.h"
#include "behavior/behavior_manager.h"
#include "behavior/behavior_test.h"
#include "controller/dwa_controller.h"
namespace dwa_local_planner {
class DWAPlannerROS : public nav_core::BaseLocalPlanner {
 public:
  DWAPlannerROS();
  ~DWAPlannerROS();
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  bool isInitialized() { return initialized_; }
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);
  // bool setPlan(const PoseStampedVector& orig_global_plan) override{};
  bool setPlan(const PoseStampedVector& orig_global_plan, TaskType path_type);
  bool isGoalReached();

 private:
  void reconfigureCB(DWAPlannerConfig& config, uint32_t level);
  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);
  void publishTransformedPlan(std::vector<geometry_msgs::PoseStamped>& path);

  NavStatusInfo ComputeVel(geometry_msgs::Twist& cmd_vel);

 private:
  tf2_ros::Buffer* tf_;  ///< @brief Used for transforming point clouds
  // for visualisation, publishers of global and local plan
  ros::Publisher g_plan_pub_, l_plan_pub_, transform_plan_pub_;
  base_local_planner::LocalPlannerUtil planner_util_;
  boost::shared_ptr<DWAPlanner> dp_;  ///< @brief The trajectory controller
  costmap_2d::Costmap2DROS* costmap_ros_;
  dynamic_reconfigure::Server<DWAPlannerConfig>* dsrv_;
  dwa_local_planner::DWAPlannerConfig default_config_;
  bool setup_;
  geometry_msgs::PoseStamped current_pose_;
  base_local_planner::LatchedStopRotateController latchedStopRotateController_;

  bool initialized_;

  base_local_planner::OdometryHelperRos odom_helper_;
  std::string odom_topic_;

 private:
  std::shared_ptr<CommonAlgorithm> common_algorithm_;
  StateManager state_manager_;
  BehaviorManager behavior_manager_;
  // Behavior
  std::shared_ptr<BehaviorIdle> idle_behavior_;
  // std::shared_ptr<BehaviorTest> test_behavior_;
  std::shared_ptr<BehaviorFixNav> fix_follow_decision_executer_;
  std::shared_ptr<BehaviorFreeNav> free_decision_executer_;
  std::shared_ptr<ChargingBehaviorExecuter> charging_behavior_executer_;
};
};  // namespace dwa_local_planner
#endif
