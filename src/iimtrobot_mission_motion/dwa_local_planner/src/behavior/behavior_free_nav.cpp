#include "behavior/behavior_free_nav.h"

BehaviorFreeNav::BehaviorFreeNav(tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros,
                                 costmap_2d::Costmap2D *costmap_2d, std::shared_ptr<DWAPlanner> &dp,
                                 std::shared_ptr<VisualizerUtil> &visualizer_util)
    : BehaviorBase(costmap_2d),
      tf_(tf),
      costmap_ros_(costmap_ros),
      costmap_2d_(costmap_2d),
      dp_(dp),
      visualizer_util_(visualizer_util) {
  state_map_fun_[FreeNavState::IDLE] = std::bind(&BehaviorFreeNav::ExecIDLE, this, std::placeholders::_1,
                                                 std::placeholders::_2, std::placeholders::_3);
  state_map_fun_[FreeNavState::GETTING_NEW_PLAN] =
      std::bind(&BehaviorFreeNav::ExecRequestNewPlan, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  state_map_fun_[FreeNavState::ROTATION_IN_PLACE] =
      std::bind(&BehaviorFreeNav::ExecRotationInPlace, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  state_map_fun_[FreeNavState::NORMAL_RUNNING] =
      std::bind(&BehaviorFreeNav::ExecNormalRunning, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  state_map_fun_[FreeNavState::MOVE_TO_GOAL] =
      std::bind(&BehaviorFreeNav::ExecMoveToFinalGoal, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  // state_map_fun_[FreeNavState::ROTATION_AT_GOAL] =
  //     std::bind(&BehaviorFreeNav::ExecRotationAtGoalPos, this, std::placeholders::_1,
  //     std::placeholders::_2,
  //               std::placeholders::_3);
  // state_map_fun_[FreeNavState::RECOVERY_STATE] =
  //     std::bind(&BehaviorFreeNav::ExecRecovery, this, std::placeholders::_1, std::placeholders::_2,
  //               std::placeholders::_3);
  path_manager_ = std::make_unique<PathManager>(tf, costmap_2d);
  CreateFsmState();
  ros::NodeHandle private_nh("~/BehaviorFreeNav/");
  private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.35);
  private_nh.param("xy_goal_precise_tolerance", xy_goal_precise_tolerance_, 0.05);
  global_frame_id_ = "odom";
}

void BehaviorFreeNav::CreateFsmState() {
  state_machine_.On(FreeNavState::IDLE, FreeNavEvent::REQUEST_ROTATION_IN_PLACE) = [&] {
    LOG(INFO) << "CreateFsmState: Request from IDLE->ROTATION_IN_PLACE";
    state_machine_.SetState(FreeNavState::ROTATION_IN_PLACE);
  };
  state_machine_.On(FreeNavState::IDLE, FreeNavEvent::REQUEST_NORMAL_RUNNING) = [&] {
    LOG(INFO) << "CreateFsmState: Request from IDLE->NORMAL_RUNNING";
    // ResetNormalRunning();
    state_machine_.SetState(FreeNavState::NORMAL_RUNNING);
  };

  // NORMAL_RUNNING State
  state_machine_.On(FreeNavState::NORMAL_RUNNING, FreeNavEvent::REQUEST_PLAN) = [&] {
    LOG(INFO) << "CreateFsmState: From NORMAL_RUNNING to GETTING_NEW_PLAN";
    state_machine_.SetState(FreeNavState::GETTING_NEW_PLAN);
  };
  state_machine_.On(FreeNavState::NORMAL_RUNNING, FreeNavEvent::REQUEST_ROTATION_IN_PLACE) = [&] {
    LOG(INFO) << "CreateFsmState: Request from NORMAL_RUNNING->ROTATION_IN_PLACE";
    state_machine_.SetState(FreeNavState::ROTATION_IN_PLACE);
  };
  state_machine_.On(FreeNavState::NORMAL_RUNNING, FreeNavEvent::REQUEST_MOVE_TO_GOAL) = [&] {
    LOG(INFO) << "CreateFsmState: Request from NORMAL_RUNNING->MOVE_TO_GOAL";
    state_machine_.SetState(FreeNavState::MOVE_TO_GOAL);
  };

  state_machine_.On(FreeNavState::RECOVERY_STATE, FreeNavEvent::REQUEST_NORMAL_RUNNING) = [&] {
    // ResetNormalRunning();
    state_machine_.SetState(FreeNavState::NORMAL_RUNNING);
  };

  state_machine_.On(FreeNavState::MOVE_TO_GOAL, FreeNavEvent::REQUEST_ROTATION_AT_GOAL) = [&] {
    LOG(INFO) << "CreateFsmState: Request from MOVE_TO_GOAL->ROTATION_AT_GOAL";
    // rotation_try_times_ = 0;
    state_machine_.SetState(FreeNavState::ROTATION_AT_GOAL);
  };
}
void BehaviorFreeNav::SetPlan(const PoseStampedVector &global_plan) {
  // 新的路径进来 重置状态
  LOG(INFO) << "free setplan";
  path_manager_->SetPlan(global_plan);
  state_machine_.SetState(FreeNavState::IDLE);
  SetGoalReached(false);
  xy_tolerance_latch_ = false;
  return;
}

void BehaviorFreeNav::ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) {
  status.status = NavStatus::STATUS_RUNNING;
  PoseStampedVector transformed_plan;
  if (!path_manager_->GetTransformedPlan(current_pose_, transformed_plan)) {
    LOG(ERROR) << "Could not get local plan";
    status.status = NavStatus::STATUS_ERROR;
    return;
  }
  visualizer_util_->publishTransformedPlan(transformed_plan);

  auto state = state_machine_.GetState();
  FreeNavEvent event = FreeNavEvent::NONE;
  if (state_map_fun_.find(state) != state_map_fun_.end()) {
    event = state_map_fun_[state](cmd_vel, status, transformed_plan);
  }
  state_machine_.Command(event);
}

FreeNavEvent BehaviorFreeNav::ExecIDLE(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                       const PoseStampedVector &transformed_plan) {
  LOG(INFO) << "BehaviorFreeNav::ExecIDLE ...";
  // todo是否需要旋转到路径方向
  return FreeNavEvent::REQUEST_NORMAL_RUNNING;
}

FreeNavEvent BehaviorFreeNav::ExecNormalRunning(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                                PoseStampedVector &transformed_plan) {
  LOG(INFO) << "BehaviorFreeNav::ExecNormalRunning ...";
  status.status = NavStatus::STATUS_RUNNING;  // default state
  // If empty plan , do nothing
  if (transformed_plan.empty()) {
    setZeroSpeed(cmd_vel);
    if (ros::Time::now() > transformed_plan_empty_time_ + ros::Duration(20.0)) {
      LOG(INFO) << "连续20s transformed_plan为空,因此结束当前任务!!!";
      status.status = NavStatus::STATUS_ERROR_RPLAN_FAILED;
    }
    return FreeNavEvent::NONE;
  } else {
    transformed_plan_empty_time_ = ros::Time::now();
  }

  dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());
  // todo 判断是否到达目标点
  if (IsGoalLatched(transformed_plan, global_frame_id_, current_pose_, xy_goal_tolerance_)) {
    LOG(INFO) << "ExecNormalRunning: REQUEST_MOVE_TO_GOAL";
    //  todo 速度平滑
    return FreeNavEvent::REQUEST_MOVE_TO_GOAL;
  }
  // todo make decision

  // DWA 轨迹打分判断模块
  if (ComputeVelWithDWA(current_pose_, cmd_vel)) {
    LOG(INFO) << "ComputeVelWithDWA";
    return FreeNavEvent::NONE;
  } else {
    status.status = NavStatus::STATUS_ERROR;
  }
  return FreeNavEvent::NONE;
}
FreeNavEvent BehaviorFreeNav::ExecRequestNewPlan(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                                 const PoseStampedVector &transformed_plan) {
  // Update status request a new plan here.
  LOG(INFO) << "ExecRequestNewPlan::request a new global plan";
  status.status = NavStatus::STATUS_REQUESTING_PLAN;
  status.last_pose_index = 0;
  return FreeNavEvent::NONE;
}

FreeNavEvent BehaviorFreeNav::ExecRotationInPlace(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                                  const PoseStampedVector &transformed_plan) {
  LOG(INFO) << "BehaviorFreeNav::ExecRotationInPlace ...";
  // todo 执行旋转
  return FreeNavEvent::REQUEST_NORMAL_RUNNING;
}
FreeNavEvent BehaviorFreeNav::ExecMoveToFinalGoal(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                                  const PoseStampedVector &transformed_plan) {
  LOG(INFO) << "ExecMoveToFinalGoal ...";
  // todo 精跟踪
  double xy_goal_precisre_tolerance = xy_goal_precise_tolerance_;
  double end_point_x = transformed_plan.back().pose.position.x;  // 获取最终的路径点坐标
  double end_point_y = transformed_plan.back().pose.position.y;
  double way_theta =
      atan2(end_point_y - current_pose_.pose.position.y, end_point_x - current_pose_.pose.position.x);
  double theta_f = angles::shortest_angular_distance(tf2::getYaw(current_pose_.pose.orientation), way_theta);
  double dis_to_goal =
      hypot(current_pose_.pose.position.x - end_point_x, current_pose_.pose.position.y - end_point_y);
  if (dis_to_goal > xy_goal_precisre_tolerance) {
    double final_speed = 0.2;
    if (theta_f == 0.0) {
      cmd_vel.linear.x = final_speed;
      cmd_vel.angular.z = 0.0;
    } else {
      double R = dis_to_goal * 0.5 / sin(theta_f);
      cmd_vel.linear.x = final_speed;
      cmd_vel.angular.z = final_speed / R;
      if (fabs(cmd_vel.angular.z) > 0.6) {
        cmd_vel.angular.z = cmd_vel.angular.z > 0.0 ? 0.6 : -0.6;
        cmd_vel.linear.x = cmd_vel.angular.z * R;
      }
    }
  } else {
    LOG(INFO) << "精跟踪到目标点";
    SetGoalReached(true);
  }

  return FreeNavEvent::NONE;
}
bool BehaviorFreeNav::ComputeVelWithDWA(geometry_msgs::PoseStamped &global_pose,
                                        geometry_msgs::Twist &cmd_vel) {
  // compute what trajectory to drive along
  geometry_msgs::PoseStamped drive_cmds;
  drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

  // call with updated footprint
  base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel_, drive_cmds);

  // pass along drive commands
  cmd_vel.linear.x = drive_cmds.pose.position.x;
  cmd_vel.linear.y = drive_cmds.pose.position.y;
  cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);
  // if we cannot move... tell someone
  std::vector<geometry_msgs::PoseStamped> local_plan;
  if (path.cost_ < 0) {
    LOG(ERROR) << "The dwa local planner failed to find a valid plan path.cost:" << path.cost_;
    local_plan.clear();
    return false;
  }
  return true;
}
bool BehaviorFreeNav::IsGoalLatched(const PoseStampedVector &global_plan, const std::string &global_frame,
                                    const geometry_msgs::PoseStamped &global_pose, double xy_goal_tolerance) {
  // we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if (!GetGoalPose(*tf_, global_plan, global_frame, goal_pose)) {
    return false;
  }
  // LOG(INFO) << "机器人当前位置：" << current_pose_;
  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;
  // check to see if we've reached the goal position

  double goal_distance = hypot(goal_x - global_pose.pose.position.x, goal_y - global_pose.pose.position.y);
  LOG(INFO) << "goal_distance: " << goal_distance;
  if (xy_tolerance_latch_ || goal_distance <= xy_goal_tolerance) {
    xy_tolerance_latch_ = true;
    return true;
  }
  return false;
}
bool BehaviorFreeNav::GetGoalPose(const tf2_ros::Buffer &tf,
                                  const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                  const std::string &global_frame, geometry_msgs::PoseStamped &goal_pose) {
  if (global_plan.empty()) {
    LOG(ERROR) << "Received plan with zero length";
    return false;
  }

  const geometry_msgs::PoseStamped &plan_goal_pose = global_plan.back();
  try {
    geometry_msgs::TransformStamped transform =
        tf.lookupTransform(global_frame, ros::Time(), plan_goal_pose.header.frame_id,
                           plan_goal_pose.header.stamp, plan_goal_pose.header.frame_id, ros::Duration(0.5));

    tf2::doTransform(plan_goal_pose, goal_pose, transform);
  } catch (tf2::LookupException &ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(),
                (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }
  return true;
}