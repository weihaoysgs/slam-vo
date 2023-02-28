#ifndef _VISUAL_ODOMETRY_HPP_
#define _VISUAL_ODOMETRY_HPP_

#include <gtest/gtest.h>

#include "Common.hpp"
#include "Config.hpp"
#include "Dataset.hpp"

namespace slam_vo {
class VisualOdometry {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<VisualOdometry> Ptr;
  VisualOdometry(const std::string &config_file_path)
      : config_file_path_(config_file_path){};
  VisualOdometry() = default;
  ~VisualOdometry() = default;

  bool VOInit();
  void Run();
  bool Step();

 private:
  const std::string config_file_path_;
  Dataset::Ptr dataset_ptr_;
};

}  // namespace slam_vo

#endif  //_VISUAL_ODOMETRY_HPP_