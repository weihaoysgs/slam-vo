#ifndef _GRONTEND_HPP_
#define _GRONTEND_HPP_

#include <opencv2/opencv.hpp>

#include "Camera.hpp"
#include "Common.hpp"
#include "Config.hpp"
#include "Frame.hpp"

namespace slam_vo {
class Frontend {
 public:
  enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<Frontend> Ptr;

  Frontend();
  ~Frontend() = default;

  bool AddNewFrame(const Frame::Ptr& frmae);
  bool StereoInit();
  int DetectFeatures();
  int FindFeatureInRightImg();
  void SetCameras(const Camera::Ptr &left_camera, const Camera::Ptr &right_camera);

 private:
  // data
  FrontendStatus front_status_ = FrontendStatus::INITING;

  Frame::Ptr current_frame_ = nullptr;
  Frame::Ptr last_frame_ = nullptr;
  Camera::Ptr camera_left_ = nullptr;
  Camera::Ptr camera_right_ = nullptr;

  // params
  int num_features_ = 200;
  int num_features_init_ = 100;
  int num_features_tracking_ = 50;
  int num_features_tracking_bad_ = 20;
  int num_features_needed_for_keyframe_ = 80;

  cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv
};

}  // namespace slam_vo

#endif  //_GRONTEND_HPP_