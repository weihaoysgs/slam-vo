#ifndef SLAMVO_MAP_HPP_
#define SLAMVO_MAP_HPP_

#include "Common.hpp"
#include "Frame.hpp"
#include "MapPoint.hpp"

namespace slam_vo {
class Map {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<Map> Ptr;
  typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandMarksType;
  typedef std::unordered_map<unsigned long, Frame::Ptr> KeyFramesType;

  Map() = default;
  ~Map() = default;

 private:
  LandMarksType landmarks_;  // 3d points
  LandMarksType active_landmarks_;
  KeyFramesType keyframes_;  // keyframes
  KeyFramesType active_keyframes_;
  Frame::Ptr current_frame_ = nullptr;
  int num_active_frames_ = 7;
  std::mutex data_mutex_;
};

}  // namespace slam_vo

#endif  // SLAMVO_MAP_HPP_