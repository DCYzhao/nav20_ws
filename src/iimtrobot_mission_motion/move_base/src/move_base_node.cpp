/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that the
 * following conditions are met:
 *
 *     * Redistributions of source code must retain the
 * above copyright notice, this list of conditions and the
 * following disclaimer.
 *     * Redistributions in binary form must reproduce the
 * above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the
 * names of its contributors may be used to endorse or
 * promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

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

  return (0);
}
