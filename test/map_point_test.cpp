//
// Created by weihao on 23-2-28.
//
#include <gtest/gtest.h>

#include "Feature.hpp"
#include "MapPoint.hpp"

using namespace slam_vo;

TEST(SLAM_VO, MapPointTest) {
  MapPoint::Ptr mappoint_ptr = std::make_shared<MapPoint>();
  Feature::Ptr feature_ptr = std::make_shared<Feature>();
  mappoint_ptr->observations_.push_back(feature_ptr);
  EXPECT_TRUE(mappoint_ptr->observations_.size());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}