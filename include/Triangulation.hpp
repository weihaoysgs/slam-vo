//
// Created by weihao on 23-3-2.
//

#ifndef SLAM_VO_TRIANGULATION_H
#define SLAM_VO_TRIANGULATION_H

#include "Common.hpp"

namespace slam_vo {

/**
 * using SVD triangulate the stereo points
 * @param poses
 * @param points
 * @param pt_world
 * @return
 */
inline bool TriangulationForStereo(const std::vector<Sophus::SE3d> &poses,
                            const std::vector<Vec3> &points, Vec3 &pt_world) {
  MatXX A(2 * poses.size(), 4);
  VecX b(2 * poses.size());
  for (size_t i = 0; i < poses.size(); i++) {
    Mat34 T = poses[i].matrix3x4();
    A.block<1, 4>(2 * i, 0) = points[i][0] * T.row(2) - T.row(0);
    A.block<1, 4>(2 * i + 1, 0) = points[i][1] * T.row(2) - T.row(1);
  }

  auto svd_result = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  pt_world =
      (svd_result.matrixV().col(3) / svd_result.matrixV()(3, 3)).head<3>();

  if (svd_result.singularValues()[3] / svd_result.singularValues()[2] < 1e-2) {
    return true;
  }
  return false;
}

}  // namespace slam_vo

#endif  // SLAM_VO_TRIANGULATION_H
