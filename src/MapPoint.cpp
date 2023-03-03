//
// Created by weihao on 23-2-28.
//
#include "MapPoint.hpp"

namespace slam_vo {
void MapPoint::RemoveObservation(std::shared_ptr<Feature>& feat) {
  std::unique_lock<std::mutex> lck(data_mutex_);
  for (auto iter = observations_.begin(); iter != observations_.end(); iter++) {
    if (iter->lock() == feat) {
      observations_.erase(iter);
      feat->map_point_.reset();
      observed_times_--;
      break;
    }
  }
}
}  // namespace slam_vo
