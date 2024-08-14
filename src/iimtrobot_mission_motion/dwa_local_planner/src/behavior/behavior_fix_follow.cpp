#include "behavior/behavior_fix_follow.h"
BehaviorFixNav::BehaviorFixNav(tf2_ros::Buffer *tf, costmap_2d::Costmap2D *costmap_2d)
    : BehaviorBase(costmap_2d), tf_(tf), costmap_2d_(costmap_2d) {
  LOG(INFO) << "BehaviorFixNav init";
  state_map_fun_[FreeNavState::IDLE] = std::bind(&BehaviorFixNav::ExecIDLE, this, std::placeholders::_1,
                                                 std::placeholders::_2, std::placeholders::_3);
  state_map_fun_[FreeNavState::GETTING_NEW_PLAN] =
      std::bind(&BehaviorFixNav::ExecRequestNewPlan, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  state_map_fun_[FreeNavState::ROTATION_IN_PLACE] =
      std::bind(&BehaviorFixNav::ExecRotationInPlace, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);
  state_map_fun_[FreeNavState::NORMAL_RUNNING] =
      std::bind(&BehaviorFixNav::ExecNormalRunning, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);

  CreateFsmState();
  ros::NodeHandle private_nh("~/BehaviorFixNav");
}

void BehaviorFixNav::CreateFsmState() {
  state_machine_.On(FreeNavState::IDLE, FreeNavEvent::REQUEST_ROTATION_IN_PLACE) = [&] {
    state_machine_.SetState(FreeNavState::NORMAL_RUNNING);
  };
  state_machine_.On(FreeNavState::IDLE, FreeNavEvent::REQUEST_NORMAL_RUNNING) = [&] {
    state_machine_.SetState(FreeNavState::ROTATION_IN_PLACE);
  };

  // ROTATION_IN_PLACE State
  state_machine_.On(FreeNavState::ROTATION_IN_PLACE, FreeNavEvent::REQUEST_NORMAL_RUNNING) = [&] {
    state_machine_.SetState(FreeNavState::NORMAL_RUNNING);
  };

  // NORMAL_RUNNING State
  state_machine_.On(FreeNavState::NORMAL_RUNNING, FreeNavEvent::REQUEST_PLAN) = [&] {
    state_machine_.SetState(FreeNavState::GETTING_NEW_PLAN);
  };
  state_machine_.On(FreeNavState::NORMAL_RUNNING, FreeNavEvent::REQUEST_ROTATION_IN_PLACE) = [&] {
    state_machine_.SetState(FreeNavState::ROTATION_IN_PLACE);
  };
  state_machine_.On(FreeNavState::NORMAL_RUNNING, FreeNavEvent::REQUEST_MOVE_TO_GOAL) = [&] {
    state_machine_.SetState(FreeNavState::MOVE_TO_GOAL);
  };

  state_machine_.On(FreeNavState::MOVE_TO_GOAL, FreeNavEvent::REQUEST_ROTATION_AT_GOAL) = [&] {
    state_machine_.SetState(FreeNavState::ROTATION_AT_GOAL);
  };
}

void BehaviorFixNav::ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) {
  // STEP 1----->获取转换后的局部路径 更新打分cost值
  PoseStampedVector transformed_plan;
  // TODO 获取转换后的局部路径
  // if (!path_manager_->GetTransformedPlan(current_pose_, transformed_plan)) {
  //   LOG(ERROR) << "Could not get local plan";
  //   status.status = NavStatus::STATUS_ERROR;
  //   return;
  // }

  auto state = state_machine_.GetState();
  FreeNavEvent event = FreeNavEvent::NONE;
  if (state_map_fun_.find(state) != state_map_fun_.end()) {
    event = state_map_fun_[state](cmd_vel, status, transformed_plan);
  }
  state_machine_.Command(event);
}

FreeNavEvent BehaviorFixNav::ExecIDLE(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                      const PoseStampedVector &transformed_plan) {
  LOG(INFO) << "BehaviorFixNav::ExecIDLE ...";
  // todo 旋转函数执行，根据旋转结果处理返回状态
  return FreeNavEvent::REQUEST_NORMAL_RUNNING;
}
FreeNavEvent BehaviorFixNav::ExecRequestNewPlan(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                                const PoseStampedVector &transformed_plan) {
  LOG(INFO) << "ExecRequestNewPlan::requset a new global plan";
  status.status = NavStatus::STATUS_REQUESTING_PLAN;
  int obs_index = 10;  // todo 计算障碍物在路径上的索引 返回给move_base
  status.last_pose_index = obs_index;
  return FreeNavEvent::NONE;
}

FreeNavEvent BehaviorFixNav::ExecRotationInPlace(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                                 const PoseStampedVector &transformed_plan) {
  LOG(INFO) << "ExecRotationInPlace: ...";
  return FreeNavEvent::REQUEST_NORMAL_RUNNING;
}

FreeNavEvent BehaviorFixNav::ExecNormalRunning(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status,
                                               PoseStampedVector &transformed_plan) {
  // todo添加ppcontrol具体算法
  FreeNavEvent event = FreeNavEvent::NONE;
  status.status = NavStatus::STATUS_RUNNING;
  // todo 判断路径碰撞
  // todo 执行ppcontrol算法
  return event;
}