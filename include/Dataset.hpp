#ifndef _DATASET_HPP_
#define _DATASET_HPP_

#include <eigen3/Eigen/Core>
#include <memory>
#include <utility>

#include "Camera.hpp"
#include "Common.hpp"

namespace slam_vo {
class Dataset {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Dataset> Ptr;
  explicit Dataset(std::string dataset_path)
      : dataset_path_(std::move(dataset_path)){};
  ~Dataset() = default;
  bool DatasetInit() const;

 private:
  const std::string dataset_path_;
  std::vector<Camera::Ptr> camera_ptr_;
};

}  // namespace slam_vo

#endif  //_DATASET_HPP_