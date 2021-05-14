

#include <realm_core/loguru.h>

#include <realm_stages/ortho_rectification.h>

using namespace realm;
using namespace stages;

OrthoRectification::OrthoRectification(const StageSettings::Ptr &stage_set, double rate)
    : StageBase("ortho_rectification", (*stage_set)["path_output"].toString(), rate, (*stage_set)["queue_size"].toInt(), bool((*stage_set)["log_to_file"].toInt())),
      m_do_publish_pointcloud((*stage_set)["publish_pointcloud"].toInt() > 0),
      m_GSD((*stage_set)["GSD"].toDouble()),
      m_settings_save({(*stage_set)["save_ortho_rgb"].toInt() > 0,
                  (*stage_set)["save_ortho_gtiff"].toInt() > 0,
                  (*stage_set)["save_elevation"].toInt() > 0,
                  (*stage_set)["save_elevation_angle"].toInt() > 0})
{
  std::cout << "Stage [" << m_stage_name << "]: Created Stage with Settings: " << std::endl;
  stage_set->print();

  registerAsyncDataReadyFunctor([=]{ return !m_buffer.empty(); });
}

void OrthoRectification::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateStatisticsIncoming();

  if (!frame->getSurfaceModel())
  {
    LOG_F(INFO, "Input frame has no surface informations. Dropping...");
    updateStatisticsBadFrame();
    return;
  }
  if (!frame->getSurfaceModel()->exists("elevation"))
  {
    LOG_F(INFO, "Input frame missing surface elevation layer. Dropping...");
    updateStatisticsBadFrame();
    return;
  }
  std::unique_lock<std::mutex> lock(m_mutex_buffer);
  m_buffer.push_back(frame);
  // Ringbuffer implementation for buffer with no pose
  if (m_buffer.size() > m_queue_size)
  {
    m_buffer.pop_front();
    updateStatisticsSkippedFrame();
  }
  notify();
}

bool OrthoRectification::process()
{
  bool has_processed = false;
  if (!m_buffer.empty())
  {
    // Prepare timing
    long t;

    Frame::Ptr frame = getNewFrame();
    LOG_F(INFO, "Processing frame #%u...", frame->getFrameId());

    // Make deep copy of the surface model, so we can resize it later on
    CvGridMap::Ptr surface_model = frame->getSurfaceModel();

    double resize_quotient = surface_model->resolution() / m_GSD;
    LOG_F(INFO, "Resize quotient rq = (elevation.resolution() / GSD) = %4.2f", resize_quotient);
    LOG_IF_F(INFO, resize_quotient < 0.9, "Loss of resolution! Consider downsizing depth map or increase GSD.");
    LOG_IF_F(INFO, resize_quotient > 1.1, "Large resizing of elevation map detected. Keep in mind that ortho resolution is now >> spatial resolution");

    // First change resolution of observed map to desired GSD
    t = getCurrentTimeMilliseconds();
    surface_model->changeResolution(m_GSD);
    LOG_F(INFO, "Timing [Resizing]: %lu ms", getCurrentTimeMilliseconds()-t);

    // Rectification needs img data, surface map and camera pose -> All contained in frame
    // Output, therefore the new additional data is written into rectified map
    t = getCurrentTimeMilliseconds();
    CvGridMap::Ptr map_rectified = ortho::rectify(frame);
    LOG_F(INFO, "Timing [Rectify]: %lu ms", getCurrentTimeMilliseconds()-t);

    // The orthophoto is contained in the rectified output. However, there is plenty of other data that is better stored
    // inside the digital surface model
    t = getCurrentTimeMilliseconds();

    CvGridMap::Ptr orthophoto = std::make_shared<CvGridMap>(map_rectified->getSubmap({"color_rgb"}));
    frame->setOrthophoto(orthophoto);

    surface_model->add("elevation_angle", (*map_rectified)["elevation_angle"]);
    surface_model->add("elevated", (*map_rectified)["elevated"]);
    surface_model->add("num_observations", (*map_rectified)["num_observations"]);

    LOG_F(INFO, "Timing [Adding]: %lu ms", getCurrentTimeMilliseconds()-t);

    // Transport results
    t = getCurrentTimeMilliseconds();
    publish(frame);
    LOG_F(INFO, "Timing [Publish]: %lu ms", getCurrentTimeMilliseconds()-t);

    // Savings every iteration
    t = getCurrentTimeMilliseconds();
    saveIter(*surface_model, *orthophoto, frame->getGnssUtm().zone, frame->getGnssUtm().band, frame->getFrameId());
    LOG_F(INFO, "Timing [Saving]: %lu ms", getCurrentTimeMilliseconds()-t);

    has_processed = true;
  }
  return has_processed;
}

void OrthoRectification::reset()
{
  // TODO: Implement
}

void OrthoRectification::saveIter(const CvGridMap& surface_model, const CvGridMap &orthophoto, uint8_t zone, char band, uint32_t id)
{
  // check for NaN
  cv::Mat valid = (surface_model["elevation"] == surface_model["elevation"]);

  if (m_settings_save.save_ortho_rgb)
    io::saveCvGridMapLayer(orthophoto, zone, band, "color_rgb", io::createFilename(m_stage_path + "/ortho/ortho_", id, ".png"));
  if (m_settings_save.save_elevation_angle)
    io::saveImageColorMap(surface_model["elevation_angle"], valid, m_stage_path + "/angle", "angle", id, io::ColormapType::ELEVATION);
  if (m_settings_save.save_ortho_gtiff)
    io::saveGeoTIFF(orthophoto.getSubmap({"color_rgb"}), zone, io::createFilename(m_stage_path + "/gtiff/gtiff_", id, ".tif"));
  if (m_settings_save.save_elevation)
    io::saveGeoTIFF(surface_model.getSubmap({"elevation"}), zone, io::createFilename(m_stage_path + "/elevation/elevation_", id, ".tif"));
}

void OrthoRectification::publish(const Frame::Ptr &frame)
{
  // First update statistics about outgoing frame rate
  updateStatisticsOutgoing();

  m_transport_frame(frame, "output/frame");
  m_transport_img((*frame->getOrthophoto())["color_rgb"], "output/rectified");

  if (m_do_publish_pointcloud)
  {
    CvGridMap::Ptr surface_model = frame->getSurfaceModel();
    CvGridMap::Ptr orthophoto = frame->getOrthophoto();

    CvGridMap map(orthophoto->roi(), orthophoto->resolution());
    map.add(*surface_model, REALM_OVERWRITE_ALL, false);
    map.add(*orthophoto, REALM_OVERWRITE_ALL, false);

    // Check for NaN
    cv::Mat valid = ((*surface_model)["elevation"] == (*surface_model)["elevation"]);

    /*cv::Mat point_cloud;
    if (frame->getSurfaceModel()->exists("elevation_normal"))
      point_cloud = cvtToPointCloud(map, "elevation", "color_rgb", "elevation_normal", "valid");
    else
      point_cloud = cvtToPointCloud(map, "elevation", "color_rgb", "", "valid");
    _transport_pointcloud(point_cloud, "output/pointcloud");*/
  }
}


Frame::Ptr OrthoRectification::getNewFrame()
{
  std::unique_lock<std::mutex> lock(m_mutex_buffer);
  Frame::Ptr frame = m_buffer.front();
  m_buffer.pop_front();
  updateStatisticsProcessedFrame();
  return (std::move(frame));
}

void OrthoRectification::initStageCallback()
{
  // If we aren't saving any information, skip directory creation
  if (!(m_log_to_file || m_settings_save.save_required()))
  {
    return;
  }

  // Stage directory first
  if (!io::dirExists(m_stage_path))
    io::createDir(m_stage_path);

  // Then sub directories
  if (!io::dirExists(m_stage_path + "/elevation") && m_settings_save.save_elevation)
    io::createDir(m_stage_path + "/elevation");
  if (!io::dirExists(m_stage_path + "/angle") && m_settings_save.save_elevation_angle)
    io::createDir(m_stage_path + "/angle");
  if (!io::dirExists(m_stage_path + "/gtiff") && m_settings_save.save_ortho_gtiff)
    io::createDir(m_stage_path + "/gtiff");
  if (!io::dirExists(m_stage_path + "/ortho") && m_settings_save.save_ortho_rgb)
    io::createDir(m_stage_path + "/ortho");
}

void OrthoRectification::printSettingsToLog()
{
  LOG_F(INFO, "### Stage process settings ###");
  LOG_F(INFO, "- GSD: %4.2f", m_GSD);
  LOG_F(INFO, "- publish_pointcloud: %i", m_do_publish_pointcloud);

  LOG_F(INFO, "### Stage save settings ###");
  LOG_F(INFO, "- save_ortho_rgb: %i", m_settings_save.save_ortho_rgb);
  LOG_F(INFO, "- save_ortho_gtiff: %i", m_settings_save.save_ortho_gtiff);
  LOG_F(INFO, "- save_elevation: %i", m_settings_save.save_elevation);
  LOG_F(INFO, "- save_elevation_angle: %i", m_settings_save.save_elevation_angle);
}

uint32_t OrthoRectification::getQueueDepth() {
  return m_buffer.size();
}