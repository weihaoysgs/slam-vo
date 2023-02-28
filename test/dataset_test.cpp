#include "Dataset.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

DEFINE_string(dataset_dir_path, "/home/weihao/code_space/dataset/kitite/00",
              "kitti dataset dir path");

TEST(SLAM_VO, DatasetTest) {
  slam_vo::Dataset::Ptr dataset_ptr =
      std::make_shared<slam_vo::Dataset>(FLAGS_dataset_dir_path);

  EXPECT_TRUE(dataset_ptr->DatasetInit());
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
