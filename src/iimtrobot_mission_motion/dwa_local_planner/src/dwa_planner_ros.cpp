#include <dwa_local_planner/dwa_planner_ros.h>

#include <pluginlib/class_list_macros.hpp>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner {

void DWAPlannerROS::reconfigureCB(DWAPlannerConfig& config, uint32_t level) {
  if (setup_ && config.restore_defaults) {
    config = default_config_;
    config.restore_defaults = false;
  }
  if (!setup_) {
    default_config_ = config;
    setup_ = true;
  }

  // update generic local planner params
  base_local_planner::LocalPlannerLimits limits;
  limits.max_vel_trans = config.max_vel_trans;
  limits.min_vel_trans = config.min_vel_trans;
  limits.max_vel_x = config.max_vel_x;
  limits.min_vel_x = config.min_vel_x;
  limits.max_vel_y = config.max_vel_y;
  limits.min_vel_y = config.min_vel_y;
  limits.max_vel_theta = config.max_vel_theta;
  limits.min_vel_theta = config.min_vel_theta;
  limits.acc_lim_x = config.acc_lim_x;
  limits.acc_lim_y = config.acc_lim_y;
  limits.acc_lim_theta = config.acc_lim_theta;
  limits.acc_lim_trans = config.acc_lim_trans;
  limits.xy_goal_tolerance = config.xy_goal_tolerance;
  limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
  limits.prune_plan = config.prune_plan;
  limits.trans_stopped_vel = config.trans_stopped_vel;
  limits.theta_stopped_vel = config.theta_stopped_vel;
  planner_util_.reconfigureCB(limits, config.restore_defaults);

  // update dwa specific configuration
  dp_->reconfigure(config);
  LOG(INFO) << "DWAPlannerROS::reconfigureCB Done";
}

DWAPlannerROS::DWAPlannerROS() : initialized_(false), odom_helper_("odom"), setup_(false) {}

void DWAPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
  if (isInitialized()) return;
  ros::NodeHandle private_nh("~/" + name);
  // g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
  // l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  // transform_plan_pub_ = private_nh.advertise<nav_msgs::Path>("transform_plan", 1);
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ros_->getRobotPose(current_pose_);

  // make sure to update the costmap we'll use for this cycle
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

  planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

  // create the actual planner that we'll use.. it'll configure itself from the parameter server
  // dp_ = std::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));
  dp_ = std::make_shared<DWAPlanner>(name, &planner_util_);
  if (private_nh.getParam("odom_topic", odom_topic_)) {
    odom_helper_.setOdomTopic(odom_topic_);
  }

  initialized_ = true;

  dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
  dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb =
      boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  visualizer_util_ = std::make_shared<VisualizerUtil>(private_nh);
  common_algorithm_ = std::make_shared<CommonAlgorithm>(tf, &costmap_2d_);
  common_algorithm_->test_log();
  // todo 初始化需要的类
  idle_behavior_ = std::make_shared<BehaviorIdle>(&costmap_2d_);
  charging_behavior_executer_ = std::make_shared<ChargingBehaviorExecuter>(tf, &costmap_2d_);
  free_decision_executer_ = std::make_shared<BehaviorFreeNav>(tf, costmap_ros, &costmap_2d_, dp_);
  fix_follow_decision_executer_ = std::make_shared<BehaviorFixNav>(tf, &costmap_2d_);

  // todo 初始化状态机
  // Behavior
  behavior_manager_.AddBehavior(CONTROLLER_STATE::IDLE, static_cast<BehaviorBase*>(idle_behavior_.get()));
  behavior_manager_.AddBehavior(CONTROLLER_STATE::FREE_PATH,
                                static_cast<BehaviorBase*>(free_decision_executer_.get()));
  behavior_manager_.AddBehavior(CONTROLLER_STATE::FIX_PATH,
                                static_cast<BehaviorBase*>(fix_follow_decision_executer_.get()));
  // behavior_manager_.AddBehavior(CONTROLLER_STATE::RECOVERY_STATE,
  //                               static_cast<BehaviorBase *>(recovery_behaviors_.get()));
  behavior_manager_.AddBehavior(CONTROLLER_STATE::CHARGING_STATE,
                                static_cast<BehaviorBase*>(charging_behavior_executer_.get()));

  state_manager_.SetState(CONTROLLER_STATE::IDLE);
  LOG(INFO) << "DWAPlannerROS Initialized";
}  // namespace dwa_local_planner

// todo setplan 下发任务需要指定任务类型 TaskType path_type
bool DWAPlannerROS::setPlan(const PoseStampedVector& orig_global_plan, TaskType path_type) {
  if (!isInitialized()) {
    LOG(ERROR) << "This planner has not been initialized, please call initialize,before using this planner";
    return false;
  }
  // if (path_type == TaskType::IDLE) {
  //   LOG(INFO) << "Get IDLE task, reste state to idle";
  //   state_manager_.SetState(CONTROLLER_STATE::IDLE);
  //   idle_behavior_->SetPlan();
  //   return true;
  // }
  // if (path_type == TaskType::FIX_FOLLOW) {  // 固定跟随
  //   state_manager_.SetState(CONTROLLER_STATE::FIX_PATH);
  //   fix_follow_decision_executer_->SetPlan(orig_global_plan);
  //   return true;
  // }
  // if (path_type == TaskType::SINGAL_GOAL) {  // 自由导航
  //   state_manager_.SetState(CONTROLLER_STATE::FREE_PATH);
  //   free_decision_executer_->SetPlan(orig_global_plan);
  //   return true;
  // }
  if (path_type == TaskType::SINGAL_GOAL) {
    state_manager_.SetState(CONTROLLER_STATE::FREE_PATH);
    free_decision_executer_->SetPlan(orig_global_plan);
  }
  // 自动充电 todo 上层setplan   任务不能一直发，要根据任务类型，要不要一直更新全局路径
  if (path_type == TaskType::CONNECT_CHARGER) {
    state_manager_.SetState(CONTROLLER_STATE::CHARGING_STATE);
    charging_behavior_executer_->SetPlan();
    charging_behavior_executer_->SetStarChargingTime(ros::Time::now());
  }
  // when we get a new plan, we also want to clear any latch we may have on goal tolerances
  latchedStopRotateController_.resetLatching();

  LOG(INFO) << "Got new plan";
  return dp_->setPlan(orig_global_plan);
}

// bool DWAPlannerROS::isGoalReached() {
//   if (!isInitialized()) {
//     LOG(ERROR) << "This planner has not been initialized, please call initialize,before using this
//     planner"; return false;
//   }
//   if (!costmap_ros_->getRobotPose(current_pose_)) {
//     LOG(ERROR) << "Could not get robot pose";
//     return false;
//   }

//   if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
//     ROS_INFO("Goal reached");
//     return true;
//   } else {
//     return false;
//   }
// }
bool DWAPlannerROS::isGoalReached() {
  if (!isInitialized()) {
    LOG(ERROR) << "dwa planner has not been initialized, please call initialize,before using this planner";
    return false;
  }

  auto state = state_manager_.GetState();
  BehaviorBase* behavior_executer = behavior_manager_.GetBehavior(state);
  if (behavior_executer) {
    return behavior_executer->IsGoalReached();
  }
}
// void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
//   base_local_planner::publishPlan(path, l_plan_pub_);
// }

// void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
//   base_local_planner::publishPlan(path, g_plan_pub_);
// }
// void DWAPlannerROS::publishTransformedPlan(std::vector<geometry_msgs::PoseStamped>& path) {
//   base_local_planner::publishPlan(path, transform_plan_pub_);
// }
DWAPlannerROS::~DWAPlannerROS() {
  // make sure to clean things up
  delete dsrv_;
}
NavStatusInfo DWAPlannerROS::ComputeVel(geometry_msgs::Twist& cmd_vel) {
  NavStatusInfo status;
  // todo更新机器人位姿状态
  // dynamic window sampling approach to get useful velocity commands
  if (!isInitialized()) {
    LOG(ERROR) << "This planner has not been initialized, please call initialize,before using this planner";
    status.status = NavStatus::STATUS_ERROR;
    return status;
  }

  if (!costmap_ros_->getRobotPose(current_pose_)) {
    LOG(ERROR) << "Could not get robot pose";
    status.status = NavStatus::STATUS_ERROR;
    return status;
  }
  behavior_manager_.UpdateRobotPose(current_pose_);
  geometry_msgs::PoseStamped robot_vel;
  odom_helper_.getRobotVel(robot_vel);
  behavior_manager_.UpdateRobotVel(robot_vel);

  UpdateCostmap2D();  // 更新地图
  auto state = state_manager_.GetState();
  BehaviorBase* behavior_executer = behavior_manager_.GetBehavior(state);
  if (behavior_executer) {
    behavior_executer->ComputeVel(cmd_vel, status);
    return status;
  }
  // todo 速度平滑
  return status;
}

void DWAPlannerROS::UpdateCostmap2D() {
  // Copy a costmap2d map to local for multi thread.
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros_->getCopiedCostmap()->getMutex()));
  costmap_2d_ = *costmap_ros_->getCopiedCostmap();
}
bool DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose,
                                               geometry_msgs::Twist& cmd_vel) {
  // dynamic window sampling approach to get useful velocity commands
  if (!isInitialized()) {
    LOG(ERROR) << "This planner has not been initialized, please call initialize,before using this planner";
    return false;
  }

  geometry_msgs::PoseStamped robot_vel;
  odom_helper_.getRobotVel(robot_vel);

  // compute what trajectory to drive along
  geometry_msgs::PoseStamped drive_cmds;
  drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

  // call with updated footprint
  base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
  // ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_,path.cost_);

  // pass along drive commands
  cmd_vel.linear.x = drive_cmds.pose.position.x;
  cmd_vel.linear.y = drive_cmds.pose.position.y;
  cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

  // if we cannot move... tell someone
  std::vector<geometry_msgs::PoseStamped> local_plan;
  if (path.cost_ < 0) {
    LOG(ERROR) << "The dwa local planner failed to find a valid plan path.cost:" << path.cost_;
    local_plan.clear();
    visualizer_util_->publishLocalPlan(local_plan);
    return false;
  }

  // Fill out the local plan
  for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
    double p_x, p_y, p_th;
    path.getPoint(i, p_x, p_y, p_th);

    geometry_msgs::PoseStamped p;
    p.header.frame_id = costmap_ros_->getGlobalFrameID();
    p.header.stamp = ros::Time::now();
    p.pose.position.x = p_x;
    p.pose.position.y = p_y;
    p.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, p_th);
    tf2::convert(q, p.pose.orientation);
    local_plan.push_back(p);
  }
  // publish information to the visualizer
  visualizer_util_->publishLocalPlan(local_plan);
  return true;
}

// TODO NavStatusInfo回传status
bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!costmap_ros_->getRobotPose(current_pose_)) {
    LOG(ERROR) << "Could not get robot pose";
    return false;
  }
  // TODO 截取路径
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
    LOG(ERROR) << "Could not get local plan";
    // 增加导航状态 status error
    return false;
  }
  visualizer_util_->publishTransformedPlan(transformed_plan);
  // if the global plan passed in is empty... we won't do anything
  if (transformed_plan.empty()) {
    LOG(ERROR) << "dwa_local_planner", "Received an empty transformed plan.";
    return false;
  }

  // 更新局部路径在地图中的cost
  dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

  // todo 判断路径朝向与机器人朝向，转到路径方向
  // TODO 根据任务类型，进入到不同控制器中执行，dwa或pp
  if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
    // publish an empty plan because we've reached our goal position
    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    visualizer_util_->publishGlobalPlan(transformed_plan);
    visualizer_util_->publishLocalPlan(local_plan);
    base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
    return latchedStopRotateController_.computeVelocityCommandsStopRotate(
        cmd_vel, limits.getAccLimits(), dp_->getSimPeriod(), &planner_util_, odom_helper_, current_pose_,
        [this](auto pos, auto vel, auto vel_samples) { return dp_->checkTrajectory(pos, vel, vel_samples); });
  } else {
    LOG(INFO) << "compute dwa vel";  //日志test
    bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
    if (isOk) {
      visualizer_util_->publishGlobalPlan(transformed_plan);
    } else {
      LOG(ERROR) << "DWA planner failed to produce path.";
      std::vector<geometry_msgs::PoseStamped> empty_plan;
      visualizer_util_->publishGlobalPlan(empty_plan);
    }
    return isOk;
  }
}
};  // namespace dwa_local_planner
