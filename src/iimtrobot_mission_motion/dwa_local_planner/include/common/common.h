#ifndef COMMON_H
#define COMMON_H
#include <common/math_util.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <glog/logging.h>
#include <nav_core/task_mode.h>

#include <functional>
#include <map>
#include <mutex>
#include <utility>
#include <vector>
typedef std::vector<geometry_msgs::PoseStamped> PoseStampedVector;

enum class PLANNING_STATE { REFRESH_STATE, GETONCE_STATE, GETONCE_ROTATE_STATE, STOP_STATE, NULL_STATE };
enum NAVIGATION_METHOD {
  FREE_NAVIGATION_MODE,   // 自由导航模式，不检测全局路径的障碍物，物流模式
  FOLLOW_NAVIGATION_MODE  // 检测全局路径上的障碍物，清扫模式
};
enum ROTATE_METHOD {
  FREE_STATE,    // 空闲状态
  BACK_RIGHT,    // 右后
  ROTATE_RIGHT,  // 原地右转
  FORWARD_LEFT,  // 前右
  BACK_LEFT,     // 后左
  ROTATE_LEFT,   // 原地左转
  FORWARD_RIGHT  // 前右
};

enum class FreeNavState : uint8_t {
  IDLE,
  GETTING_NEW_PLAN,
  ROTATION_IN_PLACE,
  ROTATION_AT_GOAL,
  NORMAL_RUNNING,
  MOVE_TO_GOAL,
  RECOVERY_STATE,
};

enum class FreeNavEvent : uint8_t {
  NONE,
  REQUEST_PLAN,
  REQUEST_ROTATION_IN_PLACE,
  REQUEST_ROTATION_AT_GOAL,
  REQUEST_NORMAL_RUNNING,
  REQUEST_MOVE_TO_GOAL,
  REQUEST_RECOVERY,
};

enum class MotionDecision : uint8_t {
  NONE,
  STOP,
  STOP_GET_PATH,
  AVOID,
};

enum class CONTROLLER_STATE {
  IDLE,
  FREE_PATH,
  FIX_PATH,
  CHARGING_READY_STATE,
  CHARGING_STATE,
  CHARGING_FINISHED,
  END_CHARGING,
  LOCALIZATION,
  RECOVERY_STATE,
  BYPASS_GOAL_STATE,
  ROTATE_STATE,
  PP_CON_STATE,
};

enum class CONTROLLER_EVENT {
  REQUEST_IDLE,
  NORMAL_STATE,
  SINGLE_GOAL_STATE,
  BYPASS_GOAL_STATE,
  CHARGING_READY_STATE,
  CHARGING_STATE,
  CHARGING_FINISHED,
  END_CHARGING,
  LOCALIZATION,
  RECOVERY_STATE,
};

struct RecoveryState {  // 旋转脱困相关参数
  bool rotate_left_set = true;
  bool rotate_right_set = false;
  bool return_to_middle = false;
  bool current_rotate_left_state = false;
  double recovery_sum_yaw = 0.0;
  double recovery_start_yaw = 0.0;
};

enum class RecoveryBehaviorType {
  NONE,
  SWAY_LEFT_RIGHT,
  MOVE_BACK,
  MOVE_TO_FREE_SPACE,
  SWAY_SLOW,
  SWAY_QUICK,
};
#endif