#ifndef BEHAVIOR_CHARGING_H
#define BEHAVIOR_CHARGING_H
#include <common_base/fsm_base.h>

#include "behavior/behavior_base.h"
#include "common/common.h"
#include "common/msg.h"
enum class ChargingState : uint8_t {
  NONE,
  WAITING_CHARGER_POSE = 0,  // 获取充电桩位置
  STATE_CORRECT_POSTURE,     // 旋转到充电桩对象
  STATE_MOVE_FORWARD,        // 前进到充电桩
  STATE_SAFETY_CHECK,        // 检测两侧是否有障碍物
  STATE_DOCKING,             // 充电成功
  STATE_FAILURE_HANDLE,      // 充电失败
  STATE_DETACHING_CHARGER,   // 充电失败

};

enum class ChargingEvent : uint8_t {
  NONE,
  REQUEST_CHARGER_POSE = 0,  // 获取充电桩位置
  REQUEST_ADJUST_DIRECTION,  // 旋转到充电桩对象
  REQUEST_MOVE_FORWARD,      // 前进到充电桩
  REQUEST_SAFETY_CHECK,      // 检测两侧是否有障碍物
  REQUEST_DOCKING,           // 充电成功
  REQUEST_FAILURE_HANDLE,    // 充电失败
  REQUEST_DETACH_CHARGER,    //
};

class ChargingBehaviorExecuter : public BehaviorBase {
 public:
  // ChargingBehaviorExecuter(tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
  ChargingBehaviorExecuter(tf2_ros::Buffer *tf, costmap_2d::Costmap2D *costmap_2d);
  ~ChargingBehaviorExecuter() = default;
  void SetPlan();
  void ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) override;

  bool endCharging(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  void ResetChargingState() { state_machine_.SetState(ChargingState::NONE); }
  void SetChargerConnectedFlag(bool flag) { charging_flag_ = flag; }
  void SetEndChargingTime(ros::Time times) { detaching_task_start_time_ = times; }
  void SetStarChargingTime(ros::Time times) { state_start_time_ = times; }

  // State call back fun
  void SetChargerRequestComand(ChargingEvent event) { return state_machine_.Command(event); }

 private:
  bool IsNearbyObsExisted();
  bool Charging_side_Obstacle(double check_point_x, double check_point_y);
  bool robot_wait(double time, geometry_msgs::Twist &cmd_vel);
  bool UpdateChargerPoint(geometry_msgs::TransformStamped &current_charge_point);  // tf获取充电桩位姿

  tf2_ros::Buffer *tf_;
  // costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_2d_;

  inline void setZeroSpeed(geometry_msgs::Twist &cmd_vel) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  char charge_recovery_max_count_ = 0;
  bool charging_flag_ = false;
  double max_linear_x_mps_, max_angular_z_rps_;

  ros::Publisher laser_detection_pub_, charging_status_pub_;

  ros::Time detaching_task_start_time_;
  bool robot_waiting_flag_ = false;
  ros::Time waiting_time_, charging_wait_obstacle_time_;
  ros::Time state_start_time_;

  std::map<ChargingState, std::function<ChargingEvent(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status)>>
      state_func_map_;

  ChargingEvent ExecuteRequestChargerPose(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  ChargingEvent ExecuteCorrectSelfPosture(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  ChargingEvent ExecuteMoveToChargerPose(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  ChargingEvent ExecuteSafetyCheck(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  ChargingEvent EexcuteDocking(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  ChargingEvent ExecuteFailCharging(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);

  ChargingEvent ExecuteDetachingCharger(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);

  FsmMachine<ChargingState, ChargingEvent> state_machine_;
  void CreateFsmState();
};

#endif