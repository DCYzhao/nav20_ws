#include <controller/pp_controller.h>

PPController::PPController() {
  ros::NodeHandle nn;

  ros::NodeHandle private_nh("~/PPController/");
  private_nh.param("rotate_to_heading_min_angle", rotate_to_heading_min_angle_, 0.95);
  private_nh.param("check_path_curvature_dist", check_path_curvature_dist_, 3.0);
  private_nh.param("use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_, true);
  private_nh.param("lookahead_time", lookahead_time_, 0.9);
  private_nh.param("forward_dist", forward_dist_, 0.6);
  LOG(INFO) << "init PP_controller";
}