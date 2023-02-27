#include "Config.hpp"
namespace slam_vo {


bool Config::SetParameterFile(const std::string &file_name) {
  if (config_ == nullptr) config_ = std::shared_ptr<Config>(new Config);
  config_->yaml_file_ =
      cv::FileStorage(file_name.c_str(), cv::FileStorage::READ);
  if (!config_->yaml_file_.isOpened()) {
    LOG(ERROR) << "Yaml Parameter File Open Failed";
    config_->yaml_file_.release();
    return false;
  }
  return true;
}

Config::~Config() {
  if (yaml_file_.isOpened()) {
    yaml_file_.release();
  }
}

std::shared_ptr<Config> Config::config_ = nullptr;

}  // namespace slam_vo