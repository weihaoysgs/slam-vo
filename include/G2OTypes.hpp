//
// Created by weihao on 23-3-4.
//

#ifndef SLAM_VO_G2OTYPES_H
#define SLAM_VO_G2OTYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <utility>

#include "Common.hpp"

namespace slam_vo {

//! pose vertex
//! R, t have 6 dim
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  void setToOriginImpl() override { _estimate = Sophus::SE3d(); }

  void oplusImpl(const double *update) override {
    // clang-format off
    Vec6 update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    // clang-format on
  }

  bool read(std::istream &in) override { return true; }
  bool write(std::ostream &out) const override { return true; }
};

//! landmark(X,Y,Z) pose
class VertexXYZ : public g2o::BaseVertex<3, Vec3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  void setToOriginImpl() override { _estimate = Vec3::Zero(); }

  void oplusImpl(const double *update) override {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
  }

  bool read(std::istream &in) override { return true; }
  bool write(std::ostream &out) const override { return true; }
};

class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeProjectionPoseOnly(Vec3 position, Mat33 K)
      : position_3d_(std::move(position)), K_(std::move(K)) {}

  void computeError() override {
    const VertexPose *v = dynamic_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Vec3 pos_pixel = K_ * (T * position_3d_);
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
  }

  void linearizeOplus() override {
    const VertexPose *v = dynamic_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    double fx = K_(0, 0);
    double fy = K_(1, 1);
    double X = position_3d_[0];
    double Y = position_3d_[1];
    double Z = position_3d_[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
        -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv, fy * Y * Zinv2,
        fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;
  }

  bool read(std::istream &in) override { return true; }
  bool write(std::ostream &out) const override { return true; }

 private:
  Vec3 position_3d_;
  Mat33 K_;
};

// clang-format off
class EdgeProjection : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeProjection(Mat33 K, const Sophus::SE3d &camera_ext)
      : K_(std::move(K)), camera_ext_(camera_ext) {}

  void computeError() override {
    const VertexPose *v0 = dynamic_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ *v1 = dynamic_cast<VertexXYZ *>(_vertices[1]);
    Sophus::SE3d T = v0->estimate();
    Vec3 pos_pixel = K_ * (camera_ext_ * (T * v1->estimate()));
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
  }

  void linearizeOplus() override {
    const VertexPose *v0 = dynamic_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ *v1 = dynamic_cast<VertexXYZ *>(_vertices[1]);
    Sophus::SE3d T = v0->estimate();
    Vec3 pw = v1->estimate();
    Vec3 pos_cam = camera_ext_ * T * pw;
    double fx = K_(0, 0);
    double fy = K_(1, 1);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;

    _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
        -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv, fy * Y * Zinv2,
        fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;

    _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                       camera_ext_.rotationMatrix() * T.rotationMatrix();
  }

 private:
  Mat33 K_;
  Sophus::SE3d camera_ext_;
};

}  // namespace slam_vo

#endif  // SLAM_VO_G2OTYPES_H
