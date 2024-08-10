#include <common/common_algorithm.h>

CommonAlgorithm::CommonAlgorithm(tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : tf_(tf), costmap_ros_(costmap_ros) {
  ros::NodeHandle nh;
  LOG(INFO) << "common_algorithm INIT";
}

void CommonAlgorithm::test_log() {
  LOG(INFO) << "using common algorithm class";
  return;
}
