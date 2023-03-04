#ifndef _GRONTEND_HPP_
#define _GRONTEND_HPP_

#include <opencv2/opencv.hpp>

#include "Camera.hpp"
#include "Common.hpp"
#include "Config.hpp"
#include "Frame.hpp"
#include "Triangulation.hpp"
#include "Map.hpp"
#include "View.hpp"
#include "G2OTypes.hpp"

namespace slam_vo {
class Frontend {
 public:
  enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<Frontend> Ptr;

  Frontend();
  ~Frontend() = default;

  bool AddNewFrame(const Frame::Ptr& frame);
  bool StereoInit();
  int DetectFeatures();
  int FindFeatureInRightImg();
  bool BuildInitMap();
  void Track();
  int TrackLastFrame();
  int EstimateCurrentPose();
  void SetCameras(const Camera::Ptr &left_camera, const Camera::Ptr &right_camera);
  void SetMap(const Map::Ptr &map) { map_ptr_ = map; };
  void SetViewer(const std::shared_ptr<View> &viewer) { viewer_ptr_ = viewer; }

 private:
  // data
  FrontendStatus front_status_ = FrontendStatus::INITING;

  Frame::Ptr current_frame_ = nullptr;
  Frame::Ptr last_frame_ = nullptr;
  Camera::Ptr camera_left_ = nullptr;
  Camera::Ptr camera_right_ = nullptr;
  Map::Ptr  map_ptr_ = nullptr;
  View::Ptr viewer_ptr_ = nullptr;

  // params
  int num_features_ = 200;
  int num_features_init_ = 100;
  int num_features_tracking_ = 50;
  int num_features_tracking_bad_ = 20;
  int num_features_needed_for_keyframe_ = 80;

  int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

  cv::Ptr<cv::GFTTDetector> gftt_;  //!  feature detector in opencv
  Sophus::SE3d relative_motion_;    //! relative motion about last frame and current frame
};

}  // namespace slam_vo

#endif  //_GRONTEND_HPP_