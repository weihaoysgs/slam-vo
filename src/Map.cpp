#include "Map.hpp"

namespace slam_vo {
void Map::InsertKeyFrame(const Frame::Ptr &frame) {
  current_frame_ = frame;
  if (keyframes_.find(frame->key_frame_id_) == keyframes_.end()) {
    keyframes_.insert(std::make_pair(frame->key_frame_id_, frame));
    active_keyframes_.insert(std::make_pair(frame->key_frame_id_, frame));
  } else {
    keyframes_[frame->key_frame_id_] = frame;
    active_keyframes_[frame->key_frame_id_] = frame;
  }
  if (active_keyframes_.size() > num_active_frames_) {
    RemoveOldKeyframes();
  }
}

void Map::RemoveOldKeyframes() {
  if (current_frame_ == nullptr) return;
  //! find the frame max far away current frame and nearest current frame
  double max_dis = 0, min_dis = 9999;
  unsigned long max_kf_id = 0, min_kf_id = 0;
  auto Twc = current_frame_->GetFramePose().inverse();
  for (auto &kf : active_keyframes_) {
    if (kf.second == current_frame_) continue;
    auto dis = (kf.second->GetFramePose() * Twc).log().norm();
    if (dis > max_dis) {
      max_dis = dis;
      max_kf_id = kf.first;
    }
    if (dis < min_dis) {
      min_dis = dis;
      min_kf_id = kf.first;
    }

    const double min_dis_th = 0.2;  // 最近阈值
    Frame::Ptr frame_to_remove = nullptr;
    if (min_dis < min_dis_th) {
      // 如果存在很近的帧，优先删掉最近的
      frame_to_remove = keyframes_.at(min_kf_id);
    } else {
      // 删掉最远的
      frame_to_remove = keyframes_.at(max_kf_id);
    }


    // remove keyframe and landmark observation
    active_keyframes_.erase(frame_to_remove->key_frame_id_);
    for (auto feat : frame_to_remove->left_features_) {
      auto mp = feat->map_point_.lock();
      if (mp) {
        mp->RemoveObservation(feat);
      }
    }
    for (auto feat : frame_to_remove->right_features_) {
      if (feat == nullptr) continue;
      auto mp = feat->map_point_.lock();
      if (mp) {
        mp->RemoveObservation(feat);
      }
    }
  }

  ClearMap();
}

void Map::ClearMap() {
  for (auto iter = active_landmarks_.begin();
       iter != active_landmarks_.end();) {
    if (iter->second->observed_times_ == 0) {
      iter = active_landmarks_.erase(iter);
    } else {
      ++iter;
    }
  }
}
}  // namespace slam_vo
