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
      std::cout << "Init success" << std::endl;
      break;
    }
    case FrontendStatus::TRACKING_BAD: {
      // code
      std::cout << "Tracking" << std::endl;
      break;
    }
    case FrontendStatus::LOST: {
      // code
      std::cout << "Lost" << std::endl;
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
  int num_tracked_features = FindFeatureInRightImg();
  if (num_tracked_features < num_features_init_) {
    return false;
  }

  //! show image
  {
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
  }

  bool build_map_success = BuildInitMap();
  if (build_map_success) {
    front_status_ = FrontendStatus::TRACKING_GOOD;
    if (viewer_ptr_) {
      viewer_ptr_->AddCurrentFrame(current_frame_);
      viewer_ptr_->UpdateMap();
    }

    return true;
  }
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
      Vec2 pt_2d = camera_left_->world2pixel(mp->GetPointPose(),
                                             current_frame_->GetFramePose());
    } else {
      pts_right.push_back(feature->position_.pt);
    }
  }
  //! calculate optical flow
  std::vector<uchar> status;
  Mat error;
  cv::calcOpticalFlowPyrLK(
      current_frame_->left_image_, current_frame_->right_image_, pts_left,
      pts_right, status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_goods_pts = 0;
  for (size_t i = 0; i < status.size(); i++) {
    if (status[i]) {
      cv::KeyPoint kpt(pts_right[i], 7);
      Feature::Ptr feat = std::make_shared<Feature>(current_frame_, kpt);
      feat->is_left_img_point = false;
      current_frame_->right_features_.push_back(feat);
      num_goods_pts++;
    } else {
      current_frame_->right_features_.push_back(nullptr);
    }
  }
  return num_goods_pts;
}

void Frontend::SetCameras(const Camera::Ptr& left_camera,
                          const Camera::Ptr& right_camera) {
  this->camera_left_ = left_camera;
  this->camera_right_ = right_camera;
}
bool Frontend::BuildInitMap() {
  std::vector<Sophus::SE3d> cameras_pose{camera_left_->GetCameraPose(),
                                         camera_right_->GetCameraPose()};
  size_t cnt_init_landmarks = 0;
  //! traverse left image feature list
  for (size_t i{0}; i < current_frame_->left_features_.size(); i++) {
    //! the point in left image no match point in right image
    if (current_frame_->right_features_[i] == nullptr) continue;
    //! create mappoint from triangulation
    std::vector<Vec3> points{
        // clang-format off
        camera_left_->pixel2camera(Vec2(current_frame_->left_features_[i]->position_.pt.x, current_frame_->left_features_[i]->position_.pt.y)),
        camera_right_->pixel2camera(Vec2(current_frame_->right_features_[i]->position_.pt.x, current_frame_->right_features_[i]->position_.pt.y))
        // clang-format on
    };
    Vec3 p_world = Vec3::Zero();
    //! triangulation the point
    if (TriangulationForStereo(cameras_pose, points, p_world) &&
        p_world[2] > 0) {
      auto new_mappoint = MapPoint::CreateNewMappoint();
      new_mappoint->SetPointPose(p_world);
      new_mappoint->AddObservation(current_frame_->left_features_[i]);
      new_mappoint->AddObservation(current_frame_->right_features_[i]);
      current_frame_->left_features_[i]->map_point_ = new_mappoint;
      current_frame_->right_features_[i]->map_point_ = new_mappoint;
      cnt_init_landmarks++;
    }
    current_frame_->SetKeyFrame();
    map_ptr_->InsertKeyFrame(current_frame_);
    //    backend_->UpdateMap();
  }
  if (cnt_init_landmarks > num_features_init_) {
    return true;
  } else {
    return false;
  }
}
}  // namespace slam_vo
