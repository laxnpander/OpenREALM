

#ifndef OPENREALM_DUMMY_REFERENCER_H
#define OPENREALM_DUMMY_REFERENCER_H

#include <realm_vslam_base/geospatial_referencer_IF.h>

namespace realm
{

class DummyReferencer : public GeospatialReferencerIF
{
public:
  explicit DummyReferencer(const cv::Mat &T_c2g);

  void init(const std::vector<Frame::Ptr> &frames) override;
  cv::Mat getTransformation() override;
  void update(const Frame::Ptr &frame) override;
  bool isBuisy() override;
  bool isInitialized() override;

private:

  std::mutex m_mutex_t_c2g;
  cv::Mat m_transformation_c2g;
};

} // namespace realm

#endif //OPENREALM_DUMMY_REFERENCER_H
