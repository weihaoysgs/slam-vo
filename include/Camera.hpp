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
  //! coordinate transform: world, camera, pixel
  Vec3 world2camera(const Vec3 &p_w, const Sophus::SE3d &T_c_w);
  Vec3 camera2world(const Vec3 &p_c, const Sophus::SE3d &T_c_w);
  Vec2 camera2pixel(const Vec3 &p_c) const;
  Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);
  Vec3 pixel2world(const Vec2 &p_p, const Sophus::SE3d &T_c_w, double depth = 1);
  Vec2 world2pixel(const Vec3 &p_w, const Sophus::SE3d &T_c_w);

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
