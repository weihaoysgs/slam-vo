#ifndef _FRAME_HPP_
#define _FRAME_HPP_

#include <memory>

#include "Common.hpp"
#include "Feature.hpp"

namespace slam_vo {

// forward declare
class Feature;

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Frame> Ptr;

  // clang-format off
  unsigned long int id_;            	// frame id
  unsigned long int key_frame_id_;  	// the distribute keyframe id (if keyframe)
  double timestamp_;			// timestamp
  bool is_keyframe_{false};             // if keyframe
  cv::Mat left_image_, right_image_;    // stereo image
  Sophus::SE3d pose_;                   // frame pose
  std::mutex pose_mutex_;  		// in different thread, we will update the frame pose
  std::vector<std::shared_ptr<Feature>> left_features_;   // left image features
  std::vector<std::shared_ptr<Feature>> right_features_;  // right images features

  // clang-format on
 public:
  Frame() = default;
  Frame(unsigned long id, double timestamp, Sophus::SE3d &pose,
        cv::Mat &left_image, cv::Mat &right_image)
      : id_(id),
        timestamp_(timestamp),
        pose_(pose),
        left_image_(left_image),
        right_image_(right_image){};
  ~Frame() = default;

  Sophus::SE3d GetFramePose() {
    std::unique_lock<std::mutex> lck(pose_mutex_);
    return pose_;
  }
  void SetFramePose(Sophus::SE3d &pose) {
    std::unique_lock<std::mutex> lck(pose_mutex_);
    pose_ = pose;
  }

  void SetKeyFrame() {
    static unsigned long factory_keyframe_id_ = 0;
    this->is_keyframe_ = true;
    this->key_frame_id_ = factory_keyframe_id_++;
  }

  static std::shared_ptr<Frame> CreateNewFrame() {
    static unsigned long factory_frame_id_ = 0;
    Frame::Ptr new_frame = std::make_shared<Frame>();
    new_frame->id_ = factory_frame_id_++;
    return new_frame;
  }

 private:
};

}  // namespace slam_vo

#endif  // _FRAME_HPP_
