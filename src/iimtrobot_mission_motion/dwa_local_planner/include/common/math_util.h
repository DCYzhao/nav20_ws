#ifndef MATH_UTIL_H
#define MATH_UTIL_H
#include <geometry_msgs/PoseStamped.h>

#include <cmath>
namespace math_util {
double CalDistM(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);
double getGoalPositionDistance(const geometry_msgs::PoseStamped &global_pose, double goal_x, double goal_y);
double sum(double x, double y);
int sign(double x);
}  // namespace math_util
#endif