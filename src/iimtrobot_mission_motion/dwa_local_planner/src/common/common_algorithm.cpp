#include <common/common_algorithm.h>

CommonAlgorithm::CommonAlgorithm(tf2_ros::Buffer* tf, costmap_2d::Costmap2D* costmap_2d)
    : tf_(tf), costmap_2d_(costmap_2d) {
  ros::NodeHandle nh;
  LOG(INFO) << "common_algorithm INIT";
}

void CommonAlgorithm::test_log() {
  LOG(INFO) << "using common algorithm class";
  return;
}
