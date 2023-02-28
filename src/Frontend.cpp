#include "Frontend.hpp"

namespace slam_vo {

Frontend::Frontend() {
  gftt_ = cv::GFTTDetector::create(
      Config::GetYamlParamByKey<int>("num_features"), 0.01, 20.0);
  num_features_ = Config::GetYamlParamByKey<int>("num_features");
  num_features_init_ = Config::GetYamlParamByKey<int>("num_features_init");
}

bool Frontend::AddNewFrame(const Frame::Ptr& frame) {
  current_frame_ = frame;
  switch (front_status_) {
    case FrontendStatus::INITING: {
      // code
      break;
    }
    case FrontendStatus::TRACKING_GOOD: {
      // code
      break;
    }
    case FrontendStatus::TRACKING_BAD: {
      // code
      break;
    }
    case FrontendStatus::LOST: {
      // code
      break;
    }

    default:
      break;
  }
  last_frame_ = current_frame_;
  return true;
}

bool Frontend::StereoInit() {
  // code
  return true;
}

int Frontend::DetectFeatures() {
  // create mask for feature detect
  // in this project, we assume that the feature point can not
  // change appear in (10,10) range in x,y axis,
  // is s very simple assume, but can accelerate the feature detect
  cv::Mat mask(current_frame_->left_image_.size(), CV_8UC1, 255);
  for(auto &pt: current_frame_->left_features_){
  }
}
}  // namespace slam_vo
