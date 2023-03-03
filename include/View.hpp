#ifndef SLAM_VO_VIEW_HPP_
#define SLAM_VO_VIEW_HPP_

#include <pangolin/pangolin.h>

#include <thread>

#include "Common.hpp"
#include "Map.hpp"

namespace slam_vo {
class View {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<View> Ptr;

  void SetMap(const Map::Ptr &map) { map_ = map; }

  void Close();

  void AddCurrentFrame(const Frame::Ptr &current_frame);

  void UpdateMap();

  void ThreadView();

  void DrawFrame(const Frame::Ptr &frame, const float *color);

  void DrawMapPoints();

  void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

  cv::Mat PlotFrameImage();

  View();

  ~View() = default;

 private:
  Frame::Ptr current_frame_ = nullptr;
  Map::Ptr map_;

  std::thread t_view_thread_;
  std::mutex viewer_data_mutex_;

  bool view_running_ = true;
  bool map_update_ = false;

  std::unordered_map<unsigned long, Frame::Ptr> active_keyframe_;
  std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks;
};

}  // namespace slam_vo

#endif  // SLAM_VO_VIEW_HPP_