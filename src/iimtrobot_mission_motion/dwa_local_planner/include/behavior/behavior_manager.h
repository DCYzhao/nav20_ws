
#ifndef BEHAVIOR_MANAGER_H
#define BEHAVIOR_MANAGER_H
#include <common_base/fsm_base.h>

#include "behavior/behavior_base.h"
#include "common/common.h"
#include "common/msg.h"
class BehaviorManager : public BehaviorBase {
 public:
  BehaviorManager() { LOG(INFO) << "BehaviorManager init"; }
  ~BehaviorManager() {}
  void AddBehavior(CONTROLLER_STATE state, BehaviorBase *behavior) { behavior_map_list_[state] = behavior; }
  BehaviorBase *GetBehavior(CONTROLLER_STATE state) {
    if (behavior_map_list_.find(state) != behavior_map_list_.end()) {
      return behavior_map_list_[state];
    }
    return nullptr;
  }
  virtual void UpdateRobotVel(geometry_msgs::PoseStamped &robot_vel) override {
    for (auto &behavior : behavior_map_list_) {
      behavior.second->UpdateRobotVel(robot_vel);
    }
  }
  virtual void UpdateRobotPose(geometry_msgs::PoseStamped &current_pose) override {
    for (auto &behavior : behavior_map_list_) {
      behavior.second->UpdateRobotPose(current_pose);
    }
  }

 private:
  std::map<CONTROLLER_STATE, BehaviorBase *> behavior_map_list_;
};
#endif