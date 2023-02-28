#ifndef _FEATURE_HPP_
#define _FEATURE_HPP_

#include <utility>

#include "Common.hpp"
#include "Frame.hpp"

namespace slam_vo {

class Frame;
class MapPoint;

class Feature {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Feature> Ptr;

  std::weak_ptr<Frame> belong_frame_;  // this point belong to which frame
  cv::KeyPoint position_;              // feature point position information
  std::weak_ptr<MapPoint> map_point_;  // associate map point
  bool is_outlier{false};              // if abnormal point, default false
  bool is_left_img_point{true};        // if left image point, default true

  Feature(const std::shared_ptr<Frame>& belong_frame,
          cv::KeyPoint feature_point)
      : belong_frame_(belong_frame), position_(std::move(feature_point)) {}
  Feature() = default;
  ~Feature() = default;

 private:
};

}  // namespace slam_vo

#endif  // _FEATURE_HPP_
