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

#include <realm_densifier_base/plane_sweep.h>

using namespace realm;
using namespace densifier;

PlaneSweep::PlaneSweep(const DensifierSettings::Ptr &settings)
: _nrof_frames((uint8_t)(*settings)["n_cams"].toInt()), _resizing((*settings)["resizing"].toDouble())
{
  assert(_nrof_frames > 1);
  settings->print();
  _settings.enable_subpix            =  (*settings)["enable_subpix"].toInt() > 0;
  _settings.enable_color_match       =  (*settings)["enable_color_match"].toInt() > 0;
  _settings.enable_out_best_depth    =  (*settings)["enable_out_best_depth"].toInt() > 0;
  _settings.enable_out_best_cost     =  (*settings)["enable_out_best_cost"].toInt() > 0;
  _settings.enable_out_cost_vol      =  (*settings)["enable_out_cost_vol"].toInt() > 0;
  _settings.enable_out_uniq_ratio    =  (*settings)["enable_out_uniq_ratio"].toInt() > 0;
  _settings.nrof_planes              =  (*settings)["nrof_planes"].toInt();
  _settings.scale                    =  (*settings)["scale"].toDouble();
  _settings.match_window_size.width  =  (*settings)["match_window_size_x"].toInt();
  _settings.match_window_size.height =  (*settings)["match_window_size_y"].toInt();
  _settings.occlusion_mode           = (PSL::PlaneSweepOcclusionMode)       (*settings)["occlusion_mode"].toInt();
  _settings.plane_gen_mode           = (PSL::PlaneSweepPlaneGenerationMode) (*settings)["plane_gen_mode"].toInt();
  _settings.match_cost               = (PSL::PlaneSweepMatchingCosts)       (*settings)["match_cost"].toInt();
  _settings.subpx_interp_mode        = (PSL::PlaneSweepSubPixelInterpMode)  (*settings)["subpx_interp_mode"].toInt();
}

cv::Mat PlaneSweep::densify(const std::deque<Frame::Ptr> &frames, uint8_t ref_idx)
{
  assert(frames.size() == _nrof_frames);

  // Get min and max scene depth for reference frame (middle frame)
  Frame::Ptr frame_ref = frames[ref_idx];
  auto min_depth = (float) frame_ref->getMinSceneDepth();
  auto max_depth = (float) frame_ref->getMaxSceneDepth();

  PSL::CudaPlaneSweep cps;
  cps.setScale(_settings.scale);
  cps.setMatchWindowSize(_settings.match_window_size.width, _settings.match_window_size.height);
  cps.setNumPlanes(_settings.nrof_planes);
  cps.setOcclusionMode(_settings.occlusion_mode);
  cps.setPlaneGenerationMode(_settings.plane_gen_mode);
  cps.setMatchingCosts(_settings.match_cost);
  cps.setSubPixelInterpolationMode(_settings.subpx_interp_mode);
  cps.enableOutputBestCosts(_settings.enable_out_best_cost);
  cps.enableOuputUniquenessRatio(_settings.enable_out_uniq_ratio);
  cps.enableOutputCostVolume(_settings.enable_out_cost_vol);
  cps.setZRange(min_depth, max_depth);

  if (_settings.enable_out_best_cost)
    cps.enableOutputBestDepth();
  if (_settings.enable_color_match)
    cps.enableColorMatching();
  if (_settings.enable_color_match)
    cps.enableSubPixel();

  // Create psl cameras for densification and feed to plane sweep handle
  int ref_id_psl = 0;
  for (uint32_t i = 0; i < frames.size(); ++i)
  {
    // Get frame from container and prepare
    Frame::Ptr frame = frames[i];
    frame->setImageResizeFactor(_resizing);

    // Use resized image grayscale
    cv::Mat img = frame->getResizedImageUndistorted();
    cv::Mat img_valid = fixImageType(img);
    //cv::cvtColor(img, img, CV_BGR2GRAY);

    // Convert to PSL style camera
    PSL::CameraMatrix<double> cam = convertToPslCamera(frames[i]->getResizedCamera());

    // Feed image to plane sweep handle and safe id
    int id = cps.addImage(img_valid, cam);

    // Reference idx will be used to identify reference frame
    if (i == ref_idx)
      ref_id_psl = id;
  }

  // Now start processing for reference frame
  try
  {
    cps.process(ref_id_psl);
  }
  catch (const PSL::Exception &e)
  {

  }

  // Get depthmap
  PSL::DepthMap<float, double> depth_map_psl = cps.getBestDepth();

  return convertToCvMat(depth_map_psl);
}

uint8_t PlaneSweep::getNrofInputFrames()
{
  return _nrof_frames;
}

double PlaneSweep::getResizeFactor()
{
  return _resizing;
}

cv::Mat PlaneSweep::convertToCvMat(PSL::DepthMap<float, double> &depth_map_psl)
{
  uint32_t height = depth_map_psl.getHeight();
  uint32_t width = depth_map_psl.getWidth();
  cv::Mat_<float> depths_mat(height, width, depth_map_psl.getDataPtr());
  depths_mat.convertTo(depths_mat, CV_32F);
  return depths_mat.clone();
}

cv::Mat PlaneSweep::fixImageType(const cv::Mat &img)
{
  cv::Mat img_fixed;
  if (_settings.enable_color_match)
  {
    if (img.channels() == 1)
      throw(std::invalid_argument("Error fixing image type: Image has only one channel, no color matching possible!"));
    else
      img_fixed = img;
  }
  else
  {
    if (img.channels() == 4)
      cv::cvtColor(img, img_fixed, CV_BGRA2BGR);
    else
      img_fixed = img;
  }
  return img_fixed;
}

PSL::CameraMatrix<double> PlaneSweep::convertToPslCamera(const camera::Pinhole::Ptr &cam)
{
  // Convert calibration
  cv::Mat cv_K = cam->K();
  Eigen::Matrix<double, 3, 3> K = Eigen::Matrix3d::Identity();
  K(0, 0) = cv_K.at<double>(0, 0);
  K(1, 1) = cv_K.at<double>(1, 1);
  K(0, 2) = cv_K.at<double>(0, 2);
  K(1, 2) = cv_K.at<double>(1, 2);

  // Transformation defined as camera to world
  cv::Mat cv_T_w2c = cam->Tw2c();

  // Convert rotation mat
  cv::Mat cv_R = cv_T_w2c.rowRange(0, 3).colRange(0, 3);
  Eigen::Matrix<double, 3, 3> R = Eigen::Matrix3d(3, 3);
  R << cv_R.at<double>(0, 0), cv_R.at<double>(0, 1), cv_R.at<double>(0, 2),
       cv_R.at<double>(1, 0), cv_R.at<double>(1, 1), cv_R.at<double>(1, 2),
       cv_R.at<double>(2, 0), cv_R.at<double>(2, 1), cv_R.at<double>(2, 2);

  // Convert translation vec
  cv::Mat cv_t = cv_T_w2c.rowRange(0, 3).col(3);
  Eigen::Matrix<double, 3, 1> t;
  t << cv_t.at<double>(0), cv_t.at<double>(1), cv_t.at<double>(2);

  return PSL::CameraMatrix<double>(K, R, t);
}

void PlaneSweep::printSettingsToLog()
{
  LOG_F(INFO, "### PlaneSweep settings ###");
  LOG_F(INFO, "- nframes: %i", _nrof_frames);
  LOG_F(INFO, "- enable_subpix: %i", _settings.enable_subpix);
  LOG_F(INFO, "- enable_color_match: %i", _settings.enable_color_match);
  LOG_F(INFO, "- enable_out_best_depth: %i", _settings.enable_out_best_depth);
  LOG_F(INFO, "- enable_out_best_cost: %i", _settings.enable_out_best_cost);
  LOG_F(INFO, "- enable_out_cost_vol: %i", _settings.enable_out_cost_vol);
  LOG_F(INFO, "- enable_out_uniq_ratio: %i", _settings.enable_out_uniq_ratio);
  LOG_F(INFO, "- nrof_planes: %i", _settings.nrof_planes);
  LOG_F(INFO, "- scale: %2.2f", _settings.scale);
  LOG_F(INFO, "- match_window_size_width: %i", _settings.match_window_size.width);
  LOG_F(INFO, "- match_window_size_height: %i", _settings.match_window_size.height);
  LOG_F(INFO, "- occlusion_mode: %i", static_cast<int>(_settings.occlusion_mode));
  LOG_F(INFO, "- plane_gen_mode: %i", static_cast<int>(_settings.plane_gen_mode));
  LOG_F(INFO, "- match_cost: %i", static_cast<int>(_settings.match_cost));
  LOG_F(INFO, "- subpx_interp_mode: %i", static_cast<int>(_settings.subpx_interp_mode));


}