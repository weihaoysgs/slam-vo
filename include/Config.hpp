#ifndef _CONFIG_HPP_
#define _CONFIG_HPP_

#include <glog/logging.h>

#include <memory>
#include <opencv2/opencv.hpp>

namespace slam_vo {
class Config {
 public:
  ~Config();
  template <typename T>
  static T GetYamlParamByKey(const std::string &key) {
    return T(Config::config_->yaml_file_[key]);
  }
  // set a new config file
  static bool SetParameterFile(const std::string &filename);

 private:
  Config() = default;
  static std::shared_ptr<Config> config_;
  cv::FileStorage yaml_file_;
};

}  // namespace slam_vo

#endif  //_CONFIG_HPP_
