#include <move_base/move_base.h>
#include <tf2_ros/transform_listener.h>

#include "glog/logging.h"
// 将信息输出到单独的文件和 LOG(ERROR)
void SignalHandle(const char *data, int size) {
  ros::shutdown();
  std::ofstream fs("/home/iimt/logdir/.navigation_log", std::ios::app | std::ios::out);
  std::string str = std::string(data, size);
  LOG(ERROR) << str;
  fs << str;
  fs.close();
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::InstallFailureWriter(&SignalHandle);
  FLAGS_logbufsecs = 0;       // 日志实时输出
  FLAGS_max_log_size = 10;    // 最大日志文件大小 10M
  FLAGS_logtostderr = false;  // 设置日志消息是否转到标准输出而不是日志文件

  FLAGS_stop_logging_if_full_disk = true;  // 设置是否在磁盘已满时避免日志记录到磁盘
  FLAGS_colorlogtostderr = true;           // 设置记录到标准输出的颜色消息（如果终端支持）
  FLAGS_alsologtostderr = true;            // 设置日志消息除了日志文件之外是否去标准输出
  if (access("/home/iimt/logdir/move_base/", 0) == -1) {
    std::string fp = "mkdir -p ";
    fp = fp + "/home/iimt/logdir/move_base/";
    int res = system(fp.c_str());
  }
  google::SetLogDestination(google::INFO, "/home/iimt/logdir/move_base/move_base");
  LOG(INFO) << "Glog ON";

  ros::init(argc, argv, "move_base_node");
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  move_base::MoveBase move_base(buffer);

  // ros::MultiThreadedSpinner s;
  ros::spin();
  google::ShutdownGoogleLogging();
  return (0);
}
