/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <realm_common/loguru.hpp>

#include <realm_stages/mosaicing.h>

using namespace realm;
using namespace stages;

Mosaicing::Mosaicing(const StageSettings::Ptr &stage_set)
    : StageBase("mosaicing", stage_set->get<std::string>("path_output"), stage_set->get<int>("queue_size")),
      _utm_reference(nullptr),
      _publish_mesh_nth_iter(0),
      _publish_mesh_every_nth_kf(stage_set->get<int>("publish_mesh_every_nth_kf")),
      _do_publish_mesh_at_finish(stage_set->get<int>("publish_mesh_at_finish") > 0),
      _downsample_publish_mesh(stage_set->get<double>("downsample_publish_mesh")),
      _use_surface_normals(true),
      _th_elevation_min_nobs(stage_set->get<int>("th_elevation_min_nobs")),
      _th_elevation_var((float)stage_set->get<double>("th_elevation_variance")),
      _settings_save({stage_set->get<int>("save_valid") > 0,
                      stage_set->get<int>("save_ortho_rgb_one") > 0,
                      stage_set->get<int>("save_ortho_rgb_all") > 0,
                      stage_set->get<int>("save_ortho_gtiff_one") > 0,
                      stage_set->get<int>("save_ortho_gtiff_all") > 0,
                      stage_set->get<int>("save_elevation_one") > 0,
                      stage_set->get<int>("save_elevation_all") > 0,
                      stage_set->get<int>("save_elevation_var_one") > 0,
                      stage_set->get<int>("save_elevation_var_all") > 0,
                      stage_set->get<int>("save_elevation_obs_angle_one") > 0,
                      stage_set->get<int>("save_elevation_obs_angle_all") > 0,
                      stage_set->get<int>("save_elevation_mesh_one") > 0,
                      stage_set->get<int>("save_num_obs_one") > 0,
                      stage_set->get<int>("save_num_obs_all") > 0,
                      stage_set->get<int>("save_dense_ply") > 0})
{
  std::cout << "Stage [" << _stage_name << "]: Created Stage with Settings: " << std::endl;
  stage_set->print();
}

void Mosaicing::addFrame(const Frame::Ptr &frame)
{
  // First update statistics about incoming frame rate
  updateFpsStatisticsIncoming();

  if (frame->getObservedMap()->empty())
  {
    LOG_F(INFO, "Input frame missing observed map. Dropping!");
    return;
  }
  std::unique_lock<std::mutex> lock(_mutex_buffer);
  _buffer.push_back(frame);

  // Ringbuffer implementation for buffer with no pose
  if (_buffer.size() > _queue_size)
    _buffer.pop_front();
}

bool Mosaicing::process()
{
  bool has_processed = false;
  if (!_buffer.empty())
  {
    // Prepare output of incremental map update
    CvGridMap::Ptr map_update;

    Frame::Ptr frame = getNewFrame();
    CvGridMap::Ptr observed_map = frame->getObservedMap();

    LOG_F(INFO, "Processing frame #%llu...", frame->getFrameId());

    // Use surface normals only if setting was set to true AND actual data has normals
    _use_surface_normals = (_use_surface_normals && observed_map->exists("elevation_normal"));

    if (_utm_reference == nullptr)
      _utm_reference = std::make_shared<UTMPose>(frame->getGnssUtm());
    if (_global_map == nullptr)
    {
      LOG_F(INFO, "Initializing global map...");
      _global_map = observed_map;
      (*_global_map).add("elevation_var", cv::Mat::ones(_global_map->size(), CV_32F)*consts::getNoValue<float>());
      (*_global_map).add("elevation_hyp", cv::Mat::ones(_global_map->size(), CV_32F)*consts::getNoValue<float>());

      // Incremental update is equal to global map on initialization
      map_update = _global_map;
    }
    else
    {
      LOG_F(INFO, "Adding new map data to global map...");
      (*_global_map).add(*observed_map, REALM_OVERWRITE_ZERO, true);

      CvGridMap::Overlap overlap = _global_map->getOverlap(*observed_map);
      if (overlap.first == nullptr && overlap.second == nullptr)
      {
        LOG_F(INFO, "No overlap detected. Add without blending...");
      }
      else
      {
        LOG_F(INFO, "Overlap detected. Add with blending...");
        CvGridMap overlap_blended = blend(&overlap);
        (*_global_map).add(overlap_blended, REALM_OVERWRITE_ALL, false);
        cv::Rect2d roi = overlap_blended.roi();
        LOG_F(INFO, "Overlap region: [%4.2f, %4.2f] [%4.2f x %4.2f]", roi.x, roi.y, roi.width, roi.height);
        LOG_F(INFO, "Overlap area: %6.2f", roi.area());
      }

      LOG_F(INFO, "Extracting updated map...");
      map_update = std::make_shared<CvGridMap>(_global_map->getSubmap({"color_rgb", "elevation", "valid"}, observed_map->roi()));
    }

    // Publishings every iteration
    LOG_F(INFO, "Publishing...");
    publish(frame, _global_map, map_update, frame->getTimestamp());

    // Savings every iteration
    saveIter(frame->getFrameId());

    has_processed = true;
  }
  return has_processed;
}

CvGridMap Mosaicing::blend(CvGridMap::Overlap *overlap)
{
  // Overlap between global mosaic (ref) and new data (inp)
  CvGridMap ref = *overlap->first;
  CvGridMap inp = *overlap->second;

  // Data layers to grab from reference
  std::vector<std::string> ref_layers;
  // Data layers to grab from input map
  std::vector<std::string> inp_layers;

  // Surface normal computation is optional, therefore use only if set
  if (_use_surface_normals)
  {
    ref_layers = {"elevation", "elevation_normal", "elevation_var", "elevation_hyp", "elevation_angle", "elevated", "color_rgb", "num_observations", "valid"};
    inp_layers = {"elevation", "elevation_normal", "elevation_angle", "elevated", "color_rgb", "valid"};
  }
  else
  {
    ref_layers = {"elevation", "elevation_var", "elevation_hyp", "elevation_angle", "elevated", "color_rgb", "num_observations", "valid"};
    inp_layers = {"elevation", "elevation_angle", "elevated", "color_rgb", "valid"};
  }

  GridQuickAccess::Ptr ref_grid_element = std::make_shared<GridQuickAccess>(ref_layers, ref);
  GridQuickAccess::Ptr inp_grid_element = std::make_shared<GridQuickAccess>(inp_layers, inp);

  cv::Size size = ref.size();
  for (int r = 0; r < size.height; ++r)
    for (int c = 0; c < size.width; ++c)
    {
      // Move the quick access element to current position
      ref_grid_element->move(r, c);
      inp_grid_element->move(r, c);

      // Check cases for input
      if (*inp_grid_element->valid == 0)
        continue;
      if (*ref_grid_element->elevated && !*inp_grid_element->elevated)
        continue;

      if (*ref_grid_element->nobs == 0 || (*inp_grid_element->elevated && !*ref_grid_element->elevated))
        setGridElement(ref_grid_element, inp_grid_element);
      else
        updateGridElement(ref_grid_element, inp_grid_element);

    }

  return ref;
}

void Mosaicing::updateGridElement(const GridQuickAccess::Ptr &ref, const GridQuickAccess::Ptr &inp)
{
  // Formulas avr+std_dev: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
  assert(*inp->valid == 255);

  if (!*inp->elevated)
  {
    *ref->ele = *inp->ele;
    *ref->elevated = 0;
    *ref->valid = 255;
    *ref->nobs = (*ref->nobs)+(uint16_t)1;
    if (fabsf(*inp->angle - 90) < fabsf(*ref->angle - 90))
    {
      *ref->angle = *inp->angle;
      *ref->rgb = *inp->rgb;
    }
    return;
  }

  // First assume grid element is not valid and set only if legitimate values were computed.
  *ref->valid = 0;

  // First compute new variance of elevation WITH input elevation and check if it is below threshold
  // If yes, update average elevation for grid element and return
  // If no, check if hypothesis is valid
  float variance_new = ((*ref->nobs) + 1 - 2) / (float) ((*ref->nobs) + 1 - 1) * (*ref->var)
      + (*inp->ele - *ref->ele)*(*inp->ele - *ref->ele) / (float) ((*ref->nobs) + 1);
  if (variance_new < _th_elevation_var)
  {
    *ref->ele = (*ref->ele) + ((*inp->ele) - (*ref->ele))/ (float)(*ref->nobs+1);
    *ref->var = variance_new;
    *ref->nobs = (*ref->nobs)+(uint16_t)1;
    if (_use_surface_normals)
      *ref->normal = (*ref->normal) + ((*inp->normal) - (*ref->normal))/ (float)(*ref->nobs+1);
    if (*ref->nobs >= _th_elevation_min_nobs)
      *ref->valid = 255;
    // Color blending
    if (fabsf(*inp->angle - 90) < fabsf(*ref->angle - 90))
    {
      *ref->angle = *inp->angle;
      *ref->rgb = *inp->rgb;
    }
    return;
  }

  // Compute std deviation of elevation hypothesis WITH input elevation. Check then if below threshold.
  // If yes OR hypothesis is better than set elevation, switch hypothesis and elevation, update std deviation
  // If no, go on
  if (*ref->hyp > consts::getNoValue<float>())
  {
    float variance_hyp = ((*inp->ele)-(*ref->hyp))*((*inp->ele)-(*ref->hyp)) / 2.0f;
    if (variance_hyp < _th_elevation_var || variance_hyp < variance_new)
    {
      float elevation = (*ref->ele);
      *ref->var = variance_hyp;
      *ref->ele = ((*ref->hyp) + (*inp->ele))/2.0f;
      *ref->hyp = elevation;
      *ref->nobs = 2;
      if (_use_surface_normals)
        *ref->normal = ((*ref->normal) + (*inp->normal))/2.0f;
      if (*ref->nobs >= _th_elevation_min_nobs)
        *ref->valid = 255;
      *ref->angle = *inp->angle;
      *ref->rgb = *inp->rgb;
    }
  }

  // No valid assumption of grid element can be identified.
  // Set current input as new hypothesis and choose the one with the lowest variance.
  *ref->hyp = *inp->ele;
}

void Mosaicing::setGridElement(const GridQuickAccess::Ptr &ref, const GridQuickAccess::Ptr &inp)
{
  assert(*inp->valid == 255);
  *ref->ele = *inp->ele;
  *ref->elevated = *inp->elevated;
  *ref->var = 0.0f;
  *ref->hyp = 0.0f;
  *ref->angle = *inp->angle;
  *ref->rgb = *inp->rgb;
  *ref->nobs = 1;
  *ref->valid = 255;
  if (_use_surface_normals)
    *ref->normal = *inp->normal;
}

void Mosaicing::saveIter(uint32_t id)
{
  if (_settings_save.save_valid)
    io::saveImage((*_global_map)["valid"], _stage_path + "/valid", "valid", id);
  if (_settings_save.save_ortho_rgb_all)
    io::saveImage((*_global_map)["color_rgb"], _stage_path + "/ortho", "ortho", id);
  if (_settings_save.save_elevation_all)
    io::saveImageColorMap((*_global_map)["elevation"], (*_global_map)["valid"], _stage_path + "/elevation/color_map", "elevation", id, io::ColormapType::ELEVATION);
  if (_settings_save.save_elevation_var_all)
    io::saveImageColorMap((*_global_map)["elevation_var"], (*_global_map)["valid"], _stage_path + "/variance", "variance", id,io::ColormapType::ELEVATION);
  if (_settings_save.save_elevation_obs_angle_all)
    io::saveImageColorMap((*_global_map)["elevation_angle"], (*_global_map)["valid"], _stage_path + "/obs_angle", "angle", id, io::ColormapType::ELEVATION);
  if (_settings_save.save_num_obs_all)
    io::saveImageColorMap((*_global_map)["num_observations"], (*_global_map)["valid"], _stage_path + "/nobs", "nobs", id, io::ColormapType::ELEVATION);
  if (_settings_save.save_ortho_gtiff_all)
    io::saveGeoTIFF(*_global_map, "color_rgb", _utm_reference->zone, _stage_path + "/ortho", "ortho", id);
}

void Mosaicing::saveAll()
{
  // 2D map output
  if (_settings_save.save_ortho_rgb_one)
    io::saveImage((*_global_map)["color_rgb"], _stage_path + "/ortho", "ortho");
  if (_settings_save.save_elevation_one)
    io::saveImageColorMap((*_global_map)["elevation"], (*_global_map)["valid"], _stage_path + "/elevation/color_map", "elevation", io::ColormapType::ELEVATION);
  if (_settings_save.save_elevation_var_one)
    io::saveImageColorMap((*_global_map)["elevation_var"], (*_global_map)["valid"], _stage_path + "/variance", "variance", io::ColormapType::ELEVATION);
  if (_settings_save.save_elevation_obs_angle_one)
    io::saveImageColorMap((*_global_map)["elevation_angle"], (*_global_map)["valid"], _stage_path + "/obs_angle", "angle", io::ColormapType::ELEVATION);
  if (_settings_save.save_num_obs_one)
    io::saveImageColorMap((*_global_map)["num_observations"], (*_global_map)["valid"], _stage_path + "/nobs", "nobs", io::ColormapType::ELEVATION);
  if (_settings_save.save_num_obs_one)
    io::saveGeoTIFF(*_global_map, "num_observations", _utm_reference->zone, _stage_path + "/nobs", "nobs");
  if (_settings_save.save_ortho_gtiff_one)
    io::saveGeoTIFF(*_global_map, "color_rgb", _utm_reference->zone, _stage_path + "/ortho", "ortho");
  if (_settings_save.save_elevation_one)
    io::saveGeoTIFF(*_global_map, "elevation", _utm_reference->zone, _stage_path + "/elevation/gtiff", "elevation");

  // 3D Point cloud output
  if (_settings_save.save_dense_ply)
  {
    if (_global_map->exists("elevation_normal"))
      io::saveElevationPointsToPLY(*_global_map, "elevation", "elevation_normal", "color_rgb", "valid", _stage_path + "/elevation/ply", "elevation");
    else
      io::saveElevationPointsToPLY(*_global_map, "elevation", "", "color_rgb", "valid", _stage_path + "/elevation/ply", "elevation");
  }

  // 3D Mesh output
  if (_settings_save.save_elevation_mesh_one)
  {
    std::vector<cv::Point2i> vertex_ids = _mesher->buildMesh(*_global_map, "valid");
    if (_global_map->exists("elevation_normal"))
      io::saveElevationMeshToPLY(*_global_map, vertex_ids, "elevation", "elevation_normal", "color_rgb", "valid", _stage_path + "/elevation/mesh", "elevation");
    else
      io::saveElevationMeshToPLY(*_global_map, vertex_ids, "elevation", "", "color_rgb", "valid", _stage_path + "/elevation/mesh", "elevation");
  }
}

void Mosaicing::reset()
{
  LOG_F(INFO, "Reseted!");
}

void Mosaicing::finishCallback()
{
  // First polish results
  runPostProcessing();

  // Trigger savings
  saveAll();

  // Publish final mesh at the end
  if (_do_publish_mesh_at_finish)
    _transport_mesh(createMeshFaces(_global_map), "output/mesh");
}

void Mosaicing::runPostProcessing()
{

}

Frame::Ptr Mosaicing::getNewFrame()
{
  std::unique_lock<std::mutex> lock(_mutex_buffer);
  Frame::Ptr frame = _buffer.front();
  _buffer.pop_front();
  return (std::move(frame));
}

void Mosaicing::initStageCallback()
{
  // Stage directory first
  if (!io::dirExists(_stage_path))
    io::createDir(_stage_path);

  // Then sub directories
  if (!io::dirExists(_stage_path + "/elevation"))
    io::createDir(_stage_path + "/elevation");
  if (!io::dirExists(_stage_path + "/elevation/color_map"))
    io::createDir(_stage_path + "/elevation/color_map");
  if (!io::dirExists(_stage_path + "/elevation/ply"))
    io::createDir(_stage_path + "/elevation/ply");
  if (!io::dirExists(_stage_path + "/elevation/pcd"))
    io::createDir(_stage_path + "/elevation/pcd");
  if (!io::dirExists(_stage_path + "/elevation/mesh"))
    io::createDir(_stage_path + "/elevation/mesh");
  if (!io::dirExists(_stage_path + "/elevation/gtiff"))
    io::createDir(_stage_path + "/elevation/gtiff");
  if (!io::dirExists(_stage_path + "/obs_angle"))
    io::createDir(_stage_path + "/obs_angle");
  if (!io::dirExists(_stage_path + "/variance"))
    io::createDir(_stage_path + "/variance");
  if (!io::dirExists(_stage_path + "/ortho"))
    io::createDir(_stage_path + "/ortho");
  if (!io::dirExists(_stage_path + "/nobs"))
    io::createDir(_stage_path + "/nobs");
  if (!io::dirExists(_stage_path + "/valid"))
    io::createDir(_stage_path + "/valid");
}

void Mosaicing::printSettingsToLog()
{
  LOG_F(INFO, "### Stage process settings ###");
  LOG_F(INFO, "- publish_mesh_nth_iter: %i", _publish_mesh_nth_iter);
  LOG_F(INFO, "- publish_mesh_every_nth_kf: %i", _publish_mesh_every_nth_kf);
  LOG_F(INFO, "- do_publish_mesh_at_finish: %i", _do_publish_mesh_at_finish);
  LOG_F(INFO, "- downsample_publish_mesh: %4.2f", _downsample_publish_mesh);
  LOG_F(INFO, "- use_surface_normals: %i", _use_surface_normals);
  LOG_F(INFO, "- th_elevation_min_nobs: %i", _th_elevation_min_nobs);
  LOG_F(INFO, "- th_elevation_var: %4.2f", _th_elevation_var);

  LOG_F(INFO, "### Stage save settings ###");
  LOG_F(INFO, "- save_valid: %i", _settings_save.save_valid);
  LOG_F(INFO, "- save_ortho_rgb_one: %i", _settings_save.save_ortho_rgb_one);
  LOG_F(INFO, "- save_ortho_rgb_all: %i", _settings_save.save_ortho_rgb_all);
  LOG_F(INFO, "- save_ortho_gtiff_one: %i", _settings_save.save_ortho_gtiff_one);
  LOG_F(INFO, "- save_ortho_gtiff_all: %i", _settings_save.save_ortho_gtiff_all);
  LOG_F(INFO, "- save_elevation_one: %i", _settings_save.save_elevation_one);
  LOG_F(INFO, "- save_elevation_all: %i", _settings_save.save_elevation_all);
  LOG_F(INFO, "- save_elevation_var_one: %i", _settings_save.save_elevation_var_one);
  LOG_F(INFO, "- save_elevation_var_all: %i", _settings_save.save_elevation_var_all);
  LOG_F(INFO, "- save_elevation_obs_angle_one: %i", _settings_save.save_elevation_obs_angle_one);
  LOG_F(INFO, "- save_elevation_obs_angle_all: %i", _settings_save.save_elevation_obs_angle_all);
  LOG_F(INFO, "- save_elevation_mesh_one: %i", _settings_save.save_elevation_mesh_one);
  LOG_F(INFO, "- save_num_obs_one: %i", _settings_save.save_num_obs_one);
  LOG_F(INFO, "- save_num_obs_all: %i", _settings_save.save_num_obs_all);
  LOG_F(INFO, "- save_dense_ply: %i", _settings_save.save_dense_ply);
}

std::vector<Face> Mosaicing::createMeshFaces(const CvGridMap::Ptr &map)
{
  CvGridMap::Ptr mesh_sampled;
  if (_downsample_publish_mesh > 10e-6)
  {
    // Downsampling was set by the user in settings
    LOG_F(INFO, "Downsampling mesh publish to %4.2f [m/gridcell]...", _downsample_publish_mesh);
    mesh_sampled = std::make_shared<CvGridMap>(map->cloneSubmap({"elevation", "color_rgb", "valid"}));

    // TODO: Change resolution correction is not cool -> same in ortho rectification
    // Check ranges of input elevation, this is necessary to correct resizing interpolation errors
    double ele_min, ele_max;
    cv::Point2i min_loc, max_loc;
    cv::minMaxLoc((*mesh_sampled)["elevation"], &ele_min, &ele_max, &min_loc, &max_loc, (*mesh_sampled)["valid"]);

    mesh_sampled->changeResolution(_downsample_publish_mesh);

    // After resizing through bilinear interpolation there can occure bad elevation values at the border
    cv::Mat mask_low = ((*mesh_sampled)["elevation"] < ele_min);
    cv::Mat mask_high = ((*mesh_sampled)["elevation"] > ele_max);
    (*mesh_sampled)["elevation"].setTo(consts::getNoValue<float>(), mask_low);
    (*mesh_sampled)["elevation"].setTo(consts::getNoValue<float>(), mask_high);
    (*mesh_sampled)["valid"].setTo(0, mask_low);
    (*mesh_sampled)["valid"].setTo(0, mask_high);
  }
  else
  {
    LOG_F(INFO, "No downsampling of mesh publish...");
    // No downsampling was set
    mesh_sampled = map;
  }

  std::vector<cv::Point2i> vertex_ids = _mesher->buildMesh(*mesh_sampled, "valid");
  std::vector<Face> faces = cvtToMesh((*mesh_sampled), "elevation", "color_rgb", vertex_ids);
  return faces;
}

void Mosaicing::publish(const Frame::Ptr &frame, const CvGridMap::Ptr &map, const CvGridMap::Ptr &update, uint64_t timestamp)
{
  // First update statistics about outgoing frame rate
  updateFpsStatisticsOutgoing();

  _transport_img((*_global_map)["color_rgb"], "output/rgb");
  _transport_img(analysis::convertToColorMapFromCVFC1((*_global_map)["elevation"],
                                                      (*_global_map)["valid"],
                                                      cv::COLORMAP_JET), "output/elevation");
  _transport_cvgridmap(update->getSubmap({"color_rgb"}), _utm_reference->zone, _utm_reference->band, "output/update/ortho");
  //_transport_cvgridmap(update->getSubmap({"elevation", "valid"}), _utm_reference->zone, _utm_reference->band, "output/update/elevation");

  if (_publish_mesh_every_nth_kf > 0 && _publish_mesh_every_nth_kf == _publish_mesh_nth_iter)
  {
    std::vector<Face> faces = createMeshFaces(map);
    std::thread t(_transport_mesh, faces, "output/mesh");
    t.detach();
    _publish_mesh_nth_iter = 0;
  }
  else if (_publish_mesh_every_nth_kf > 0)
  {
    _publish_mesh_nth_iter++;
  }
}

Mosaicing::GridQuickAccess::GridQuickAccess(const std::vector<std::string> &layer_names, const CvGridMap &map)
: ele(nullptr),
  var(nullptr),
  hyp(nullptr),
  nobs(nullptr),
  rgb(nullptr),
  valid(nullptr)
{
  for (const auto& layer_name : layer_names)
    if (layer_name == "elevation")
      _elevation = map["elevation"];
    else if (layer_name == "elevation_normal")
      _elevation_normal = map["elevation_normal"];
    else if (layer_name == "elevation_var")
      _elevation_var = map["elevation_var"];
    else if (layer_name == "elevation_hyp")
      _elevation_hyp = map["elevation_hyp"];
    else if (layer_name == "elevation_angle")
      _elevation_angle = map["elevation_angle"];
    else if (layer_name == "color_rgb")
      _color_rgb = map["color_rgb"];
    else if (layer_name == "num_observations")
      _num_observations = map["num_observations"];
    else if (layer_name == "elevated")
      _elevated = map["elevated"];
    else if (layer_name == "valid")
      _valid = map["valid"];
    else
      throw(std::out_of_range("Error creating GridQuickAccess object. Demanded layer name does not exist!"));

  assert(!_elevation.empty() && _elevation.type() == CV_32F);
  assert(!_elevation_angle.empty() && _elevation_angle.type() == CV_32F);
  assert(!_color_rgb.empty() && _color_rgb.type() == CV_8UC4);
  assert(!_elevated.empty() && _elevated.type() == CV_8UC1);
  assert(!_valid.empty() && _valid.type() == CV_8UC1);

  move(0, 0);
}

void Mosaicing::GridQuickAccess::move(int row, int col)
{
  ele = &_elevation.ptr<float>(row)[col];
  var = &_elevation_var.ptr<float>(row)[col];
  hyp = &_elevation_hyp.ptr<float>(row)[col];
  angle = &_elevation_angle.ptr<float>(row)[col];
  nobs = &_num_observations.ptr<uint16_t>(row)[col];
  rgb = &_color_rgb.ptr<cv::Vec4b>(row)[col];
  elevated = &_elevated.ptr<uchar>(row)[col];
  valid = &_valid.ptr<uchar>(row)[col];
  if (!_elevation_normal.empty())
    normal = &_elevation_normal.ptr<cv::Vec3f>(row)[col];
}