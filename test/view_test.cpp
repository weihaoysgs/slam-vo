//
// Created by weihao on 23-3-3.
//
#include <gtest/gtest.h>
#include <gflags/gflags.h>

#include "View.hpp"

using namespace slam_vo;

TEST(SLAM_VO, ViewTest){
  auto view_ptr = std::make_shared<View>();

}

int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}