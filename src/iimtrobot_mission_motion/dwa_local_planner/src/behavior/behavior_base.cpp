#include "behavior/behavior_base.h"
// bool BehaviorBase::IsGoalLatched(const tf2_ros::Buffer &tf, const PoseStampedVector &global_plan,
//                                  const std::string &global_frame,
//                                  const geometry_msgs::PoseStamped &global_pose, double xy_goal_tolerance) {
//   // we assume the global goal is the last point in the global plan
//   geometry_msgs::PoseStamped goal_pose;
//   if (!GetGoalPose(tf, global_plan, global_frame, goal_pose)) {
//     return false;
//   }
//   // check to see if we've reached the goal position
//   if (xy_tolerance_latch_ || math_util::CalDistM(goal_pose, global_pose) <= xy_goal_tolerance) {
//     xy_tolerance_latch_ = true;
//     return true;
//   }
//   return false;
// }
// bool BehaviorBase::GetGoalPose(const tf2_ros::Buffer &tf,
//                                const std::vector<geometry_msgs::PoseStamped> &global_plan,
//                                const std::string &global_frame, geometry_msgs::PoseStamped &goal_pose) {
//   if (global_plan.empty()) {
//     LOG(ERROR) << "Received plan with zero length";
//     return false;
//   }

//   const geometry_msgs::PoseStamped &plan_goal_pose = global_plan.back();
//   try {
//     geometry_msgs::TransformStamped transform =
//         tf.lookupTransform(global_frame, ros::Time(), plan_goal_pose.header.frame_id,
//                            plan_goal_pose.header.stamp, plan_goal_pose.header.frame_id,
//                            ros::Duration(0.5));

//     tf2::doTransform(plan_goal_pose, goal_pose, transform);
//   } catch (tf2::LookupException &ex) {
//     ROS_ERROR("No Transform available Error: %s\n", ex.what());
//     return false;
//   } catch (tf2::ConnectivityException &ex) {
//     ROS_ERROR("Connectivity Error: %s\n", ex.what());
//     return false;
//   } catch (tf2::ExtrapolationException &ex) {
//     ROS_ERROR("Extrapolation Error: %s\n", ex.what());
//     if (global_plan.size() > 0)
//       ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(),
//                 (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

//     return false;
//   }
//   return true;
// }
