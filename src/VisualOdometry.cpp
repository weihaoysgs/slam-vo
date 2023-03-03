#include "VisualOdometry.hpp"

namespace slam_vo {
bool VisualOdometry::VOInit() {
  // create singleton of config file
  CHECK_EQ(Config::SetParameterFile(config_file_path_), true);
  dataset_ptr_ = std::make_shared<Dataset>(
      Config::GetYamlParamByKey<std::string>("dataset_dir"));

  CHECK_EQ(dataset_ptr_->DatasetInit(), true);
  frontend_ptr_ = std::make_shared<Frontend>();
  map_ptr_ = std::make_shared<Map>();
  viewer_ptr_ = std::make_shared<View>();
  LOG(INFO) << "VO exit";

  frontend_ptr_->SetCameras(dataset_ptr_->GetCamera(0),
                            dataset_ptr_->GetCamera(1));
  frontend_ptr_->SetMap(map_ptr_);
  frontend_ptr_->SetViewer(viewer_ptr_);
  viewer_ptr_->SetMap(map_ptr_);

  return true;
}
void VisualOdometry::Run() {
  while (true) {
    if (!Step()) {
      LOG(INFO) << "Step Error";
      break;
    } else {
      LOG(INFO) << "OK";
    }
  }

  LOG(INFO) << "VO exit";
}
bool VisualOdometry::Step() {
  Frame::Ptr new_frame = dataset_ptr_->NextFrame();
  if (new_frame == nullptr) return false;
  frontend_ptr_->AddNewFrame(new_frame);
  return true;
}
}  // namespace slam_vo