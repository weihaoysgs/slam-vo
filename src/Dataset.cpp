#include "Dataset.hpp"

namespace slam_vo {

bool Dataset::DatasetInit() {
  std::ifstream fin(dataset_path_ + "/calib.txt");
  if (!fin) {
    LOG(ERROR) << "Can not open " << dataset_path_
               << "/cal.txt, check the config file path";
    return false;
  }
  for (int i{0}; i < 4; i++) {
    char camera_name[3];
    for (char &k : camera_name) {
      fin >> k;
    }

    double projection_data[12];  // T matrix of 3x4
    for (double &k : projection_data) {
      fin >> k;
    }

    Mat33 K;
    Vec3 t;
    K << projection_data[0], projection_data[1], projection_data[2],
        projection_data[4], projection_data[5], projection_data[6],
        projection_data[8], projection_data[9], projection_data[10];
    t << projection_data[3], projection_data[7], projection_data[11];
    // How to build the camera K, t
    // The internal reference K of the two grayscale cameras is the same.
    // Here is the operation to restore the pixel value to the metric unit,
    // which is actually (1/fx * t[0]). 1/fx = K.inverse()(0,0)
    // ref: https://zhuanlan.zhihu.com/p/99114433
    t = K.inverse() * t;
    // Why multiply by 0.5 here? After experimenting, after commenting out this
    // line, the program can still run normally, but it seems to run faster.
    // you can also change the value to 0.6 or 0.3
    // Solution: because we resize the image using scale (0.5, 0.5)
    K = K * 0.5;

    Camera::Ptr new_camera =
        std::make_shared<Camera>(K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(),
                                 Sophus::SE3d(Sophus::SO3d(), t));
    cameras_ptr_.push_back(new_camera);
    LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
  }
  fin.close();
  current_image_index_ = 0;
  return true;
}

}  // namespace slam_vo
