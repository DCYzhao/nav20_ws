#ifndef TASK_MODE_H_
#define TASK_MODE_H_

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>

enum class StuckTimeLevel {
  NORMAL = 0,
  SHORT, // 困住2分钟
  LONG,  // 困住12分钟
};

enum class TaskType {
  UNKNOWN = 0,
  IDLE,
  REMAINED, // 补扫路径
  AREA,     // 覆盖路径
  FIX_FOLLOW,
  FOLLOW,
  SINGAL_GOAL, // 自由导航点任务
  AREA_GOAL,   // 区域路径前的自由导航点
  CHARGE_GOAL, // 充电的目标点
  CONNECT_CHARGER,
  DETACH_CHARGER, // 出充电桩
  RELOCATION,     // 重定位任务
};

typedef std::vector<geometry_msgs::PoseStamped> PoseStampedVector;
typedef std::vector<geometry_msgs::Pose2D> Pose2DVector;

struct PathType {
  TaskType task_type;
  PoseStampedVector poses;
};

typedef std::vector<PathType> PathSeqType;

enum class NavStatus {
  STATUS_IDLE = 0,
  STATUS_RUNNING = 1,
  STATUS_NO_LOCAL_GOAL = 2,
  STATUS_REQUESTING_PLAN = 3,
  STATUS_RETREATING = 4,
  STATUS_REACHED_GOAL = 5,
  STATUS_CLEARING = 6,
  STATUS_WAITING = 7,
  STATUS_MOVINGBACK = 8,
  STATUS_ROTATING = 9,
  STATUS_ROTATING_CLEARING = 10,
  STATUS_DWATING = 11,
  STATUS_STUCKSHORT_FIRST_TRIGGER = 12,
  STATUS_STUCKSHORT_MULTI_TRIGGER = 13,
  STATUS_STUCKLONG = 14,
  STATUS_CHARGING = 15,
  STATUS_REQUESTING_AVOIDING = 16,

  SYATEM_WARN = 200,
  SYSTEM_PAUSE = 201,

  STATUS_ERROR = 300,
  STATUS_ERROR_EMPTYLOCALPLAN,
  STATUS_ERROR_DWAFAILED,
  STATUS_ERROR_NO_CHARGER,
  STATUS_ERROR_CHARGER_FAILED,
  STATUS_ERROR_APLAN_FAILED,
  STATUS_ERROR_RPLAN_FAILED,
  STATUS_ERROR_OBSF_FAILED,
  STATUS_ERROR_WAITING_FAILED,

};

struct NavStatusInfo {
  NavStatus status;
  int last_pose_index;
};

enum PATH_MODE {
  PATH_MODE_LOCALIZATION = 0,
  PATH_MODE_FREE_LINE,
  PATH_MODE_CLEANING_AREA,
  PATH_MODE_CHARGING,
  PATH_MODE_END_CHARGING,
  PATH_MODE_WELT_FOLLOW,
};

enum TASK_STATE {
  TASK_READY = 0,
  TASK_PAUSING,
  TASK_RUNNING,
  TASK_ABORT,
};

#endif