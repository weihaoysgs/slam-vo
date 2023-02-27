#include "Config.hpp"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "Common.hpp"

DEFINE_string(yaml_config_file, "../config/kitti_config.yaml",
              "yaml config file");

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  slam_vo::InitGlog("config_test");
  slam_vo::Config::SetParameterFile(FLAGS_yaml_config_file);
  LOG(INFO) << slam_vo::Config::GetYamlParamByKey<std::string>("dataset_dir");
  return 0;
}
