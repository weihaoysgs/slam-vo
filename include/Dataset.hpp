#ifndef _DATASET_HPP_
#define _DATASET_HPP_

#include <boost/format.hpp>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <utility>
#include <gtest/gtest.h>

#include "Camera.hpp"
#include "Common.hpp"
#include "Frame.hpp"

namespace slam_vo {
class Dataset {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<Dataset> Ptr;
  //! constructor function
  explicit Dataset(std::string dataset_path)
      : dataset_path_(std::move(dataset_path)){};

  ~Dataset() = default;
  //! init dataset class
  bool DatasetInit();
  //! get next frame form dataset
  Frame::Ptr NextFrame();
  //! get camera by id
  Camera::Ptr GetCamera(const int camera_id) const {
    return cameras_ptr_[camera_id];
  }

 private:
  const std::string dataset_path_;
  std::vector<Camera::Ptr> cameras_ptr_;
  int current_image_index_{0};
};

}  // namespace slam_vo

#endif  //_DATASET_HPP_