#include "View.hpp"

namespace slam_vo {
View::View() {
  t_view_thread_ = std::thread([this] { ThreadView(); });
}

void View::ThreadView() {
  //! https://blog.csdn.net/weixin_45929038/article/details/122904705
  pangolin::CreateWindowAndBind("SLAM_VO", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC0_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState vis_camera(
      pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

  pangolin::View& vis_display =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
          .SetHandler(new pangolin::Handler3D(vis_camera));

  const float green[3] = {0, 1, 0};

  while (!pangolin::ShouldQuit() && view_running_) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0, 1.0, 1.0, 1.0);
    vis_display.Activate(vis_camera);

    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    if (current_frame_) {
      DrawFrame(current_frame_, green);
      FollowCurrentFrame(vis_camera);
    }
    if (map_) {
      DrawMapPoints();
    }
//    std::cout << "Hello View." << std::endl;
    pangolin::FinishFrame();
    usleep(5000);
  }
}

void View::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
  Sophus::SE3d Twc = current_frame_->GetFramePose().inverse();
  pangolin::OpenGlMatrix m(Twc.matrix());
  vis_camera.Follow(m);
}

void View::DrawFrame(const Frame::Ptr& frame, const float* color) {
  Sophus::SE3d Twc = frame->GetFramePose().inverse();
  const float sz = 1.0;
  const int line_width = 2.0;
  const float fx = 400;
  const float fy = 400;
  const float cx = 512;
  const float cy = 384;
  const float width = 1080;
  const float height = 768;

  glPushMatrix();

  Sophus::Matrix4f m = Twc.matrix().template cast<float>();
  glMultMatrixf((GLfloat*)m.data());

  if (color == nullptr) {
    glColor3f(1, 0, 0);
  } else
    glColor3f(color[0], color[1], color[2]);

  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glEnd();
  glPopMatrix();
}
void View::DrawMapPoints() {
  const float red[3] = {1.0, 0, 0};
  for (auto& kf : active_keyframe_) {
    DrawFrame(kf.second, red);
  }

  glPointSize(2);
  glBegin(GL_POINTS);
  for (auto& landmark : active_landmarks) {
    auto pos = landmark.second->GetPointPose();
    glColor3f(red[0], red[1], red[2]);
    glVertex3d(pos[0], pos[1], pos[2]);
  }
  glEnd();
}

void View::UpdateMap() {
  std::unique_lock<std::mutex> lck(viewer_data_mutex_);
  assert(map_ != nullptr);
  active_keyframe_ = map_->GetActiveKeyFrames();
  active_landmarks = map_->GetActiveMapPoints();
  map_update_ = true;
}

void View::AddCurrentFrame(const Frame::Ptr& current_frame) {
  std::unique_lock<std::mutex> lck(viewer_data_mutex_);
  current_frame_ = current_frame;
}
}  // namespace slam_vo
