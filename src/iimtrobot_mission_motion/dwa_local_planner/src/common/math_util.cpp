#include "common/math_util.h"
namespace math_uit {
double CalDistM(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2) {
  return hypot(pose1.pose.position.x - pose2.pose.position.x, pose1.pose.position.y - pose2.pose.position.y);
}
int sign(double x) {
  if (x < 0)
    return -1;
  else if (x == 0)
    return 0;
  else
    return 1;
}
}  // namespace math_uit
