#include "behavior/behavior_charging.h"
ChargingBehaviorExecuter::ChargingBehaviorExecuter(tf2_ros::Buffer *tf, costmap_2d::Costmap2D *costmap_2d)
    : BehaviorBase(costmap_2d), tf_(tf), costmap_2d_(costmap_2d) {
  LOG(INFO) << "ChargingBehaviorExecuter init";
  ros::NodeHandle nn;
  charging_status_pub_ = nn.advertise<std_msgs::UInt16>("charging_status", 1);
  ros::NodeHandle nh("~/ChargingBehaviorExecuter");
  nh.param("max_linear_x", max_linear_x_mps_, 0.1);
  nh.param("max_angular_z", max_angular_z_rps_, 0.35);

  state_func_map_[ChargingState::WAITING_CHARGER_POSE] =
      std::bind(&ChargingBehaviorExecuter::ExecuteRequestChargerPose, this, std::placeholders::_1,
                std::placeholders::_2);
  state_func_map_[ChargingState::STATE_CORRECT_POSTURE] =
      std::bind(&ChargingBehaviorExecuter::ExecuteCorrectSelfPosture, this, std::placeholders::_1,
                std::placeholders::_2);
  state_func_map_[ChargingState::STATE_MOVE_FORWARD] =
      std::bind(&ChargingBehaviorExecuter::ExecuteMoveToChargerPose, this, std::placeholders::_1,
                std::placeholders::_2);

  // state_func_map_[ChargingState::STATE_DETACHING_CHARGER] = std::bind(
  //     &ChargingBehaviorExecuter::ExecuteDetachingCharger, this, std::placeholders::_1,
  //     std::placeholders::_2);
  CreateFsmState();
}
void ChargingBehaviorExecuter::CreateFsmState() {
  state_machine_.On(ChargingState::NONE, ChargingEvent::REQUEST_CHARGER_POSE) = [&] {
    LOG(INFO) << "Get charger pose 1111";
    state_machine_.SetState(ChargingState::WAITING_CHARGER_POSE);
  };
  state_machine_.On(ChargingState::WAITING_CHARGER_POSE, ChargingEvent::REQUEST_ADJUST_DIRECTION) = [&] {
    state_machine_.SetState(ChargingState::STATE_CORRECT_POSTURE);
  };

  state_machine_.On(ChargingState::STATE_CORRECT_POSTURE, ChargingEvent::REQUEST_MOVE_FORWARD) = [&] {
    state_machine_.SetState(ChargingState::STATE_MOVE_FORWARD);
  };
  state_machine_.On(ChargingState::STATE_FAILURE_HANDLE, ChargingEvent::REQUEST_ADJUST_DIRECTION) = [&] {
    state_machine_.SetState(ChargingState::STATE_CORRECT_POSTURE);
  };
}

void ChargingBehaviorExecuter::SetPlan() {
  //   SetGoalReached(false);
  ResetChargingState();
  // SetChargerRequestComand(ChargingEvent::REQUEST_DETACH_CHARGER);
}

ChargingEvent ChargingBehaviorExecuter::ExecuteRequestChargerPose(geometry_msgs::Twist &cmd_vel,
                                                                  NavStatusInfo &status) {
  status.status = NavStatus::STATUS_RUNNING;
  ChargingEvent event = ChargingEvent::NONE;
  LOG(INFO) << "ChargingBehaviorExecuter::ExecuteRequestChargerPose ...";
  auto request_charing_pose_sec = (ros::Time::now() - state_start_time_).toSec();
  LOG(INFO) << "charging pose sec: " << request_charing_pose_sec;
  if (request_charing_pose_sec > 2.0) {
    return ChargingEvent::REQUEST_ADJUST_DIRECTION;  // 切换下个状态
  }
  // LOG(INFO) << "charging pose sec: " << request_charing_pose_sec;
  return event;
}

ChargingEvent ChargingBehaviorExecuter::ExecuteCorrectSelfPosture(geometry_msgs::Twist &cmd_vel,
                                                                  NavStatusInfo &status) {
  status.status = NavStatus::STATUS_ROTATING;
  LOG(INFO) << "ChargingBehaviorExecuter::ExecuteCorrectSelfPosture ...";
  auto request_charing_pose_sec = (ros::Time::now() - state_start_time_).toSec();
  LOG(INFO) << "charging pose sec: " << request_charing_pose_sec;
  if (request_charing_pose_sec > 5.0) {
    return ChargingEvent::REQUEST_MOVE_FORWARD;  // 切换下个状态
  }
  return ChargingEvent::NONE;
}

ChargingEvent ChargingBehaviorExecuter::ExecuteMoveToChargerPose(geometry_msgs::Twist &cmd_vel,
                                                                 NavStatusInfo &status) {
  // TODO need to update every loop
  status.status = NavStatus::STATUS_RUNNING;
  LOG(INFO) << "ChargingBehaviorExecuter::ExecuteMoveToChargerPose ...";
  // return ChargingEvent::REQUEST_SAFETY_CHECK; // 切换下个状态
  SetGoalReached(true);
  return ChargingEvent::NONE;
}

void ChargingBehaviorExecuter::ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) {
  auto state = state_machine_.GetState();
  ChargingEvent event = ChargingEvent::NONE;
  if (state_func_map_.find(state) != state_func_map_.end()) {
    event = state_func_map_[state](cmd_vel, status);
  }
  state_machine_.Command(event);
}
