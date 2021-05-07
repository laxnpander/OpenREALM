#include <realm_io/mvs_export.h>
#include <realm_core/loguru.h>

#include <unordered_map>

using namespace realm::io;

void MvsExport::saveFrames(const std::vector<Frame::Ptr> &frames, const std::string &directory)
{
  Frame::Ptr first_frame = frames.front();

  UTMPose utm = first_frame->getGnssUtm();

  MvsInterface interface{};

  MvsInterface::Platform platform;
  platform.name    = "OpenREALM";

  MvsInterface::Platform::Camera camera;
  camera.name      = first_frame->getCameraId();
  camera.width     = first_frame->getCamera()->width();
  camera.height    = first_frame->getCamera()->height();
  camera.K         = first_frame->getCamera()->K();
  camera.R         = MvsInterface::Mat33d(1, 0, 0, 0, 1, 0, 0, 0, 1);
  camera.C         = MvsInterface::Pos3d(0, 0, 0);
  platform.cameras.push_back(camera);

  std::unordered_map<uint32_t, MvsInterface::Vertex> sparse_cloud_map;

  std::vector<Frame::Ptr> frames_selected;
  for (int i = 0; i < frames.size(); i++)
    frames_selected.push_back(frames[i]);

  for (int i = 0; i < frames_selected.size(); ++i)
  {
    Frame::Ptr f = frames_selected.at(i);
    cv::Mat t = f->getCamera()->t();

    MvsInterface::Platform::Pose pose;
    pose.R         = f->getCamera()->R();
    pose.C         = MvsInterface::Pos3d(t.at<double>(0) - utm.easting, t.at<double>(1) - utm.northing, t.at<double>(2));
    platform.poses.push_back(pose);

    std::string filename = directory + "/image_" + std::to_string(i) + ".png";
    std::cout << "filename: " << filename << std::endl;
    cv::imwrite(filename, f->getImageUndistorted());

    MvsInterface::Image img;
    img.name       = filename;
    img.cameraID   = 0;
    img.platformID = 0;
    img.poseID     = i;
    interface.images.push_back(img);

    MvsInterface::Vertex::View view;
    view.imageID    = i;
    view.confidence = 0.;

    cv::Mat points = f->getSparseCloud()->data();
    std::vector<uint32_t> point_ids = f->getSparseCloud()->getPointIds();

    for (int j = 0; j < points.rows; ++j)
    {
      uint32_t id = point_ids[j];
      auto it = sparse_cloud_map.find(id);
      if (it != sparse_cloud_map.end())
      {
        it->second.views.push_back(view);
      }
      else
      {
        MvsInterface::Vertex vertex;
        vertex.X.x = static_cast<float>(points.at<double>(j, 0) - utm.easting);
        vertex.X.y = static_cast<float>(points.at<double>(j, 1) - utm.northing);
        vertex.X.z = static_cast<float>(points.at<double>(j, 2));
        vertex.views.push_back(view);
        sparse_cloud_map[id] = vertex;
      }
    }
  }

  for (const auto& it : sparse_cloud_map)
    interface.vertices.push_back(it.second);

  interface.platforms.push_back(platform);

  MvsArchive::SerializeSave(interface, directory + "/data.mvs");
}