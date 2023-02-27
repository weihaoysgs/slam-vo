#ifndef _DATASET_HPP_
#define _DATASET_HPP_

#include <eigen3/Eigen/Core>
#include <memory>

namespace slam_vo {
class Dataset {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Dataset> Ptr;
  Dataset(std::string &dir_path);
  ~Dataset();

 private:
  const std::string dir_path_;
};

}  // namespace slam_vo

#endif  //_DATASET_HPP_