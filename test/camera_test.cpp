#include "Camera.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "Common.hpp"
#include "Config.hpp"
using namespace slam_vo;
DEFINE_string(config_file_path, "../config/kitti_config.yaml",
              "slam vo project config file path");

TEST(SLAM_VO, CameraTest) {
  EXPECT_TRUE(Config::SetParameterFile(FLAGS_config_file_path));
  auto fx = Config::GetYamlParamByKey<double>("camera.fx");
  auto fy = Config::GetYamlParamByKey<double>("camera.fy");
  auto cx = Config::GetYamlParamByKey<double>("camera.cx");
  auto cy = Config::GetYamlParamByKey<double>("camera.cy");
  EXPECT_GT(fx, 0.0);
  EXPECT_GT(fy, 0.0);
  EXPECT_GT(cx, 0.0);
  EXPECT_GT(cy, 0.0);
  std::shared_ptr<Camera> cam_ptr =
      std::make_shared<Camera>(fx, fy, cx, cy, 0.0, Sophus::SE3d());
  std::cout << "Camera Parameters: " << std::endl
            << cam_ptr->GetCameraK() << std::endl;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
