#include "VisualOdometry.hpp"

namespace slam_vo {
bool VisualOdometry::VOInit() {
  // create singleton of config file
  CHECK_EQ(Config::SetParameterFile(config_file_path_), true);
  dataset_ptr_ = std::make_shared<Dataset>(
      Config::GetYamlParamByKey<std::string>("dataset_dir"));

  CHECK_EQ(dataset_ptr_->DatasetInit(), true);
  return true;
}
void VisualOdometry::Run() {
  while (true) {
    if (!Step()) {
      LOG(INFO) << "Step Error";
      break;
    }
  }
}
bool VisualOdometry::Step() {
  Frame::Ptr new_frame = dataset_ptr_->NextFrame();
  if (new_frame == nullptr) return false;
  cv::imshow("left_image", new_frame->left_image_);
  cv::imshow("right_image", new_frame->right_image_);
  cv::waitKey(10);
  return true;
}
}  // namespace slam_vo