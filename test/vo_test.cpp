#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "VisualOdometry.hpp"

DEFINE_string(config_file_path, "../config/kitti_config.yaml",
              "system yaml config file path");

TEST(SLAM_VO, VisualOdometryTest) {
  slam_vo::VisualOdometry::Ptr vo_ptr =
      std::make_shared<slam_vo::VisualOdometry>(FLAGS_config_file_path);
  EXPECT_TRUE(vo_ptr->VOInit());
  vo_ptr->Run();
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
