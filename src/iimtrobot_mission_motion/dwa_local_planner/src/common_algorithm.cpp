#include <dwa_local_planner/common_algorithm.h>

CommonAlgorithm::CommonAlgorithm(tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : tf_(tf), costmap_ros_(costmap_ros) {
  ros::NodeHandle nh;
}

void CommonAlgorithm::test_log() {
  LOG(INFO) << "using common algorithm class";
  return;
}
