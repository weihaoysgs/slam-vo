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
      StereoInit();
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
  int num_feature_detected = DetectFeatures();

  std::vector<cv::KeyPoint> left_img_kps;
  cv::Mat drawed_kps_img;
  for (const auto& feature : current_frame_->left_features_) {
    left_img_kps.push_back(feature->position_);
  }
  cv::drawKeypoints(current_frame_->left_image_, left_img_kps, drawed_kps_img,
                    cv::Scalar(255, 255, 0));
  cv::imshow("leftImage", current_frame_->left_image_);
  cv::imshow("rightImage", current_frame_->right_image_);
  cv::imshow("drawed_kps_img", drawed_kps_img);
  cv::waitKey(10);
  return true;
}

int Frontend::DetectFeatures() {
  // create mask for feature detect
  // in this project, we assume that the feature point can not
  // change appear in (10,10) range in x-axis, y-axis,
  // is s very simple assume, but can accelerate the feature detect
  cv::Mat mask(current_frame_->left_image_.size(), CV_8UC1, 255);
  for (auto& feature : current_frame_->left_features_) {
    cv::rectangle(current_frame_->left_image_,
                  feature->position_.pt - cv::Point2f(10, 10),
                  feature->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
  }

  std::vector<cv::KeyPoint> kps;
  gftt_->detect(current_frame_->left_image_, kps, mask);
  int cnt_feature_detected = 0;
  for (cv::KeyPoint& pt : kps) {
    current_frame_->left_features_.push_back(
        std::make_shared<Feature>(current_frame_, pt));
    cnt_feature_detected++;
  }
  return cnt_feature_detected;
}
int Frontend::FindFeatureInRightImg() {
  std::vector<cv::Point2f> pts_left, pts_right;
  for (auto& feature : current_frame_->left_features_) {
    pts_left.push_back(feature->position_.pt);
    auto mp = feature->map_point_.lock();
    if (mp) {
      // use projected points as initial guess
      // cv::Point2f pt_2d = camera_left_->world2pixel(feature->map_point_, current_frame_->pose_);
    }
    else{
      pts_right.push_back(feature->position_.pt);
    }
  }

  return 0;
}

void Frontend::SetCameras(const Camera::Ptr& left_camera,
                          const Camera::Ptr& right_camera) {
  this->camera_left_ = left_camera;
  this->camera_right_ = right_camera;
}
}  // namespace slam_vo