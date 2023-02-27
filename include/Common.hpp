#ifndef _COMMON_HPP_
#define _COMMON_HPP_
#include <glog/logging.h>

namespace slam_vo {

void InitGlog(const std::string &app_name) {
  google::InitGoogleLogging(app_name.c_str());
  google::SetLogFilenameExtension("_slam_vo_");
  google::SetLogDestination(google::INFO, "../log/");
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_max_log_size = 1024;
  FLAGS_stop_logging_if_full_disk = true;
}

}  // namespace slam_vo

#endif  // _COMMON_HPP_