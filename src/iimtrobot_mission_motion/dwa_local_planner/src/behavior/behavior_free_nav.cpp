#include "behavior/behavior_free_nav.h"

BehaviorFreeNav::BehaviorFreeNav(tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    : BehaviorBase(costmap_ros), tf_(tf), costmap_ros_(costmap_ros) {
  LOG(INFO) << "BehaviorFreeNav INIT";
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
  // state_map_fun_[FreeNavState::MOVE_TO_GOAL] =
  //     std::bind(&BehaviorFreeNav::ExecMoveToFinalGoal, this, std::placeholders::_1, std::placeholders::_2,
  //               std::placeholders::_3);
  // state_map_fun_[FreeNavState::ROTATION_AT_GOAL] =
  //     std::bind(&BehaviorFreeNav::ExecRotationAtGoalPos, this, std::placeholders::_1,
  //     std::placeholders::_2,
  //               std::placeholders::_3);
  // state_map_fun_[FreeNavState::RECOVERY_STATE] =
  //     std::bind(&BehaviorFreeNav::ExecRecovery, this, std::placeholders::_1, std::placeholders::_2,
  //               std::placeholders::_3);
  CreateFsmState();
  ros::NodeHandle private_nh("~/BehaviorFreeNav/");
  global_frame_id_ = "/odom";
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
  state_machine_.On(FreeNavState::NORMAL_RUNNING, FreeNavEvent::REQUEST_RECOVERY) = [&] {
    LOG(INFO) << "CreateFsmState: Request from NORMAL_RUNNING->RECOVERY_STATE";
    state_machine_.SetState(FreeNavState::RECOVERY_STATE);
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
}

void BehaviorFreeNav::ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) {
  PoseStampedVector transformed_plan;
  //   if (!path_manager_->GetTransformedPlan(current_pose_, transformed_plan)) {
  //     LOG(ERROR) << "Could not get local plan";
  //     status.status = NavStatus::STATUS_ERROR;
  //     return;
  //   }
  status.status = NavStatus::STATUS_RUNNING;

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
  return FreeNavEvent::REQUEST_NORMAL_RUNNING;
}

FreeNavEvent BehaviorFreeNav::ExecNormalRunning(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                                PoseStampedVector &transformed_plan) {
  // default state
  LOG(INFO) << "BehaviorFreeNav::ExecNormalRunning ...";
  status.status = NavStatus::STATUS_RUNNING;
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