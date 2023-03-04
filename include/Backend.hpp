//
// Created by weihao on 23-3-4.
//

#ifndef SLAM_VO_BACKEND_H
#define SLAM_VO_BACKEND_H

#include "Camera.hpp"
#include "G2OTypes.hpp"
#include "Map.hpp"

namespace slam_vo {

class Map;

class Backend {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Backend> Ptr;

  Backend();
  void SetCamera(const Camera::Ptr &left, const Camera::Ptr &right) {
    cam_left_ = left;
    cam_right_ = right;
  }

  void SetMap(const std::shared_ptr<Map> &map) { map_ = map; }
  void UpdateMap();
  void Stop();

 private:
  void BackendLoop();
  void Optimize(Map::KeyFramesType &keyframes, Map::LandmarksType &landmarks);

  std::thread backend_thread_;
  std::mutex data_mutex_;
  std::condition_variable map_update_;
  std::atomic<bool> backend_running_;
  Map::Ptr map_;
  Camera::Ptr cam_left_, cam_right_;
};

}  // namespace slam_vo

#endif  // SLAM_VO_BACKEND_H
