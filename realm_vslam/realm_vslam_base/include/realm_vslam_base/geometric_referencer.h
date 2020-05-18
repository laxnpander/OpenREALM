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

#ifndef PROJECT_GEOMETRIC_REFERENCER_H
#define PROJECT_GEOMETRIC_REFERENCER_H

#include <memory>
#include <numeric>

#include <realm_core/loguru.h>

#include <realm_vslam_base/geospatial_referencer_IF.h>
#include <realm_core/plane_fitter.h>

namespace realm
{

class GeometricReferencer : public GeospatialReferencerIF
{
  public:
    // First Mat: GIS pose (E,N,U,Heading), Second Mat: Visual Pose
    struct SpatialMeasurement
    {
      using Ptr = std::shared_ptr<SpatialMeasurement>;
      cv::Mat first;
      cv::Mat second;
    };

  public:
    explicit GeometricReferencer(double th_error);
    void init(const std::vector<Frame::Ptr> &frames) override;
    bool isBuisy() override;
    cv::Mat getTransformation() override;
    void update(const Frame::Ptr &frame) override;
    bool isInitialized() override;

  private:

    std::mutex _mutex_is_initialized;
    bool _is_initialized;

    std::mutex _mutex_is_buisy;
    bool _is_buisy;

    size_t _prev_nrof_unique;
    double _scale;
    double _th_error;
    double _error;

    std::mutex _mutex_t_c2g;
    cv::Mat _transformation_c2g;

    std::vector<SpatialMeasurement::Ptr> _spatials;

    void setBuisy();

    void setIdle();

    void setReference(const cv::Mat &T_c2g);

    static double computeTwoPointScale(const SpatialMeasurement::Ptr &f1, const SpatialMeasurement::Ptr &f2, double th_visual);

    static double computeAverageReferenceError(const std::vector<SpatialMeasurement::Ptr> &spatials, const cv::Mat &T_c2w);

    static cv::Mat refineReference(const std::vector<SpatialMeasurement::Ptr> &frames, const cv::Mat &T_c2w, double z_weight);

    static cv::Mat applyTransformation(const cv::Mat &T, const cv::Mat &pt);
};

} // namespace realm

#endif //PROJECT_GEOMETRIC_REFERENCER_H
