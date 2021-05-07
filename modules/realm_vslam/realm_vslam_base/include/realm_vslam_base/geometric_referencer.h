

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
    explicit GeometricReferencer(double th_error, int min_nrof_frames);

    void init(const std::vector<Frame::Ptr> &frames) override;

    bool isBuisy() override;

    cv::Mat getTransformation() override;

    void update(const Frame::Ptr &frame) override;

    bool isInitialized() override;

    double computeScaleChange(const Frame::Ptr &frame) override;

  private:

    std::mutex m_mutex_is_initialized;
    bool m_is_initialized;

    std::mutex m_mutex_is_buisy;
    bool m_is_buisy;

    size_t m_prev_nrof_unique;
    double m_scale;
    double m_th_error;
    double m_error;

    int m_min_nrof_frames;

    std::mutex m_mutex_t_c2g;
    cv::Mat m_transformation_w2g;

    std::vector<SpatialMeasurement::Ptr> m_spatials;

    void setBuisy();

    void setIdle();

    void setReference(const cv::Mat &T_c2g);

    void calibrateOrientation();

    static double computeTwoPointScale(const SpatialMeasurement::Ptr &f1, const SpatialMeasurement::Ptr &f2, double th_visual);

    static double computeAverageReferenceError(const std::vector<SpatialMeasurement::Ptr> &spatials, const cv::Mat &T_c2w);

    static cv::Mat refineReference(const std::vector<SpatialMeasurement::Ptr> &frames, const cv::Mat &T_c2w, double z_weight);

    static cv::Mat applyTransformation(const cv::Mat &T, const cv::Mat &pt);
  };

} // namespace realm

#endif //PROJECT_GEOMETRIC_REFERENCER_H
