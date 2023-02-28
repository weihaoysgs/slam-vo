#ifndef _CAMERA_HPP_
#define _CAMERA_HPP_

#include "Common.hpp"

namespace slam_vo {
class Camera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Camera> Ptr;

  Camera(double fx, double fy, double cx, double cy, double base_line,
         const Sophus::SE3d &pose)
      : fx_(fx),
        fy_(fy),
        cx_(cx),
        cy_(cy),
        pose_(pose),
        pose_inv_(pose.inverse()) {
    K_ << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
  }

  ~Camera() = default;
  Mat33 GetCameraK() const { return K_; }

 private:
  Mat33 K_;
  const double fx_;
  const double fy_;
  const double cx_;
  const double cy_;
  const Sophus::SE3d pose_;
  const Sophus::SE3d pose_inv_;
};

}  // namespace slam_vo

#endif  //_CAMERA_HPP_
