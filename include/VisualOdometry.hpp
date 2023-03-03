#ifndef VISUAL_ODOMETRY_HPP_
#define VISUAL_ODOMETRY_HPP_

#include <gtest/gtest.h>

#include <utility>

#include "Common.hpp"
#include "Config.hpp"
#include "Dataset.hpp"
#include "Frontend.hpp"
#include "MapPoint.hpp"
#include "Map.hpp"
#include "View.hpp"
#include "Triangulation.hpp"

namespace slam_vo {
class VisualOdometry {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<VisualOdometry> Ptr;
  explicit VisualOdometry(std::string config_file_path)
      : config_file_path_(std::move(config_file_path)){};
  VisualOdometry() = default;
  ~VisualOdometry() = default;

  bool VOInit();
  void Run();
  bool Step();

 private:
  const std::string config_file_path_;
  Dataset::Ptr dataset_ptr_;
  Frontend::Ptr frontend_ptr_;
  View::Ptr viewer_ptr_;
  Map::Ptr map_ptr_;
};

}  // namespace slam_vo

#endif  // VISUAL_ODOMETRY_HPP_