//
// Created by weihao on 23-2-28.
//

#ifndef SLAM_VO_MAPPOINT_H
#define SLAM_VO_MAPPOINT_H

#include "Common.hpp"
#include "Feature.hpp"
#include "Frame.hpp"

namespace slam_vo {

class Feature;

class MapPoint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  //! class shared pointer
  typedef std::shared_ptr<MapPoint> Ptr;

  int observed_times_{0};
  unsigned long id_ = 0;
  bool is_outlier{false};

  std::list<std::weak_ptr<Feature>>
      observations_;  //! the observation of this 3d point

  Vec3 GetPointPose() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return pose_point_3d_;
  }

  void SetPointPose(const Vec3 &new_pose) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    pose_point_3d_ = new_pose;
  }

  void AddObservation(const std::shared_ptr<Feature> &observation) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    observations_.push_back(observation);
    observed_times_++;
  }

  std::list<std::weak_ptr<Feature>> GetObservations() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return observations_;
  }

  static MapPoint::Ptr CreateNewMappoint(){
    static unsigned long factory_id = 0;
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
  }

  MapPoint() = default;
  ~MapPoint() = default;

 private:
  std::mutex data_mutex_;
  Vec3 pose_point_3d_ = Vec3::Zero();  // position in 3d world
};
}  // namespace slam_vo

#endif  // SLAM_VO_MAPPOINT_H
