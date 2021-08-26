

#ifndef PROJECT_GEOSPATIAL_REFERENCER_IF_H
#define PROJECT_GEOSPATIAL_REFERENCER_IF_H

#include <limits>
#include <vector>
#include <memory>
#include <mutex>

#include <realm_core/frame.h>

namespace realm
{

class GeospatialReferencerIF
{
  public:
    using Ptr = std::shared_ptr<GeospatialReferencerIF>;
    using ConstPtr = std::shared_ptr<const GeospatialReferencerIF>;

  public:
    virtual void init(const std::vector<Frame::Ptr> &frames) = 0;
    virtual cv::Mat getTransformation() = 0;
    virtual void update(const Frame::Ptr &frame) = 0;
    virtual bool isBuisy() = 0;
    virtual bool isInitialized() = 0;
    virtual double computeScaleChange(const Frame::Ptr &frame) { return std::numeric_limits<double>::quiet_NaN(); };
};

} // namespace realm

#endif //PROJECT_GEOSPATIAL_REFERENCER_IF_H
