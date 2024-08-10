#ifndef BEHAVIOR_TEST_H
#define BEHAVIOR_TEST_H
#include <behavior/behavior_base.h>
class BehaviorTest : public BehaviorBase {
 public:
  BehaviorTest(costmap_2d::Costmap2DROS *costmap_ros) : BehaviorBase(costmap_ros) {
    LOG(INFO) << "BehaviorTest init";
  };
  //   BehaviorTest(costmap_2d::Costmap2DROS *costmap_ros);
  ~BehaviorTest() = default;
  // void ComputeVel(geometry_msgs::Twist &cmd_vel, NavStatusInfo &status);
  bool GetGoalPose();

 private:
};
#endif