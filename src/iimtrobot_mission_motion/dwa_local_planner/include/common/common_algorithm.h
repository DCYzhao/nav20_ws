#ifndef COMMON_ALGORITHM_H
#define COMMON_ALGORITHM_H
#include <common/msg.h>
#include <controller/dwa_controller.h>
using namespace dwa_local_planner;
class CommonAlgorithm {
 public:
  CommonAlgorithm(tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  ~CommonAlgorithm() = default;

  inline void setZeroSpeed(geometry_msgs::Twist& cmd_vel) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  void test_log();
  int speed = 10;
  double getSimPeriod() { return 0.0; }

 private:
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  boost::shared_ptr<DWAPlanner> dp_;
};

#endif