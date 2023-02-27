#include "Config.hpp"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "Common.hpp"

DEFINE_string(yaml_config_file, "../config/kitti_config.yaml",
              "yaml config file");

TEST(SLAM_VO, ConfigFileTest) {
  EXPECT_TRUE(slam_vo::Config::SetParameterFile(FLAGS_yaml_config_file));
  EXPECT_EQ(slam_vo::Config::GetYamlParamByKey<std::string>("dataset_dir"),
            "/home/weihao/code_space/dataset/kitite/00");

  EXPECT_EQ(slam_vo::Config::GetYamlParamByKey<double>("camera.fx"), 517.3);
  EXPECT_EQ(slam_vo::Config::GetYamlParamByKey<double>("camera.fy"), 516.5);
  EXPECT_EQ(slam_vo::Config::GetYamlParamByKey<double>("camera.cx"), 325.1);
  EXPECT_EQ(slam_vo::Config::GetYamlParamByKey<double>("camera.cy"), 249.7);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
