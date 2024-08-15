#include <common/path_manager.h>

PathManager::PathManager(tf2_ros::Buffer* tf, costmap_2d::Costmap2D* costmap_2d)
    : tf_(tf), costmap_2d_(costmap_2d) {
  local_frame_ = "odom";
}

bool PathManager::GetTransformedPlan(const geometry_msgs::PoseStamped& origin_robot_pose,
                                     PoseStampedVector& transformed_plan) {
  return PrunePlan(*tf_, origin_robot_pose, transformed_plan, look_ahead_dist_);
}

bool PathManager::PrunePlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& origin_robot_pose,
                            PoseStampedVector& transformed_plan, double check_obs_dis) {
  transformed_plan.clear();
  if (origin_path_.empty()) {
    LOG(ERROR) << "Received plan with zero length";
    return false;
  }
  const geometry_msgs::PoseStamped& plan_pose = origin_path_[nearest_index_];
  try {
    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform =
        tf.lookupTransform(local_frame_, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                           plan_pose.header.frame_id, ros::Duration(0.5));
    // // let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(origin_robot_pose, robot_pose, plan_pose.header.frame_id);

    // we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap_2d_->getSizeInCellsX() * costmap_2d_->getResolution() / 2.0,
                                     costmap_2d_->getSizeInCellsY() * costmap_2d_->getResolution() / 2.0);

    size_t i = std::max(static_cast<size_t>(1), nearest_index_);
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 0;

    // we need to loop to a point on the plan that is within a certain distance of the robot
    // 找到离机器人最近的点
    size_t max_size_look = std::min(nearest_index_ + 300, origin_path_.size());
    double dis_check = 0.0;
    for (size_t i = nearest_index_; i < max_size_look; i++) {
      double x_diff = robot_pose.pose.position.x - origin_path_[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - origin_path_[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist <= sq_dist_threshold) {
        nearest_index_ = i;
        dis_check = 0.0;
        break;
      } else {
        double c_dis = pow(origin_path_[i].pose.position.x - origin_path_[i - 1].pose.position.x, 2) +
                       pow(origin_path_[i].pose.position.y - origin_path_[i - 1].pose.position.y, 2);
        dis_check += c_dis;
        if (dis_check > 1.5) break;
      }
    }
    geometry_msgs::PoseStamped newer_pose;

    // now we'll transform until points are outside of our distance threshold
    while (i < (unsigned int)origin_path_.size() && sq_dist <= sq_dist_threshold) {
      const geometry_msgs::PoseStamped& pose = origin_path_[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - origin_path_[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - origin_path_[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      transformed_plan.push_back(newer_pose);
      ++i;
    }
  } catch (tf2::LookupException& ex) {
    LOG(ERROR) << "No Transform available Error: " << ex.what();
    return false;
  } catch (tf2::ConnectivityException& ex) {
    LOG(ERROR) << "Connectivity Error: " << ex.what();
    return false;
  } catch (tf2::ExtrapolationException& ex) {
    LOG(ERROR) << "Extrapolation Error: " << ex.what();
    if (!origin_path_.empty())
      LOG(ERROR) << "Global Frame:Plan Frame size " << local_frame_ << origin_path_.size();
    return false;
  }

  return true;
}

void PathManager::SetPlan(const PoseStampedVector& new_path) {
  origin_path_.clear();
  origin_path_ = new_path;
  nearest_index_ = 0;
}

void PathManager::CalFixPathLength() {
  fix_total_length_ = 0.0;
  for (int i = 0; i < origin_path_.size() - 1; i++) {
    fix_total_length_ += math_util::CalDistM(origin_path_[i], origin_path_[i + 1]);
  }
}