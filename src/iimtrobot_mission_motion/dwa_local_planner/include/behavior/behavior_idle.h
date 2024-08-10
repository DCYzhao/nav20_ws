
#ifndef BEHAVIOR_IDLE_H
#define BEHAVIOR_IDLE_H
#include <common_base/fsm_base.h>

#include "behavior/behavior_base.h"
#include "common/common.h"
#include "common/msg.h"

class BehaviorIdle : public BehaviorBase {
 public:
  BehaviorIdle(costmap_2d::Costmap2DROS *costmap_ros) : BehaviorBase(costmap_ros) {
    LOG(INFO) << "BehaviorIdle init";
  }
  ~BehaviorIdle() = default;

  void ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status) {
    dis_to_end_ = 10000.0;
    setZeroSpeed(cmd_vel);
    status.status = NavStatus::STATUS_RUNNING;
  }
  void SetPlan() { SetGoalReached(false); }
};
#endif