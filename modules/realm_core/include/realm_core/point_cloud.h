#ifndef OPENREALM_POINT_CLOUD_H
#define OPENREALM_POINT_CLOUD_H

#include <opencv2/core.hpp>
#include <memory>

namespace realm
{

class PointCloud
{
public:
  using Ptr = std::shared_ptr<PointCloud>;

public:
  PointCloud();
  PointCloud(const std::vector<uint32_t> &point_ids, const cv::Mat &data);

  bool empty();

  cv::Mat& data();

  int size();

  std::vector<uint32_t> getPointIds();

private:
  std::vector<uint32_t> m_point_ids;
  cv::Mat m_data;

};

} // namespace realm

#endif //OPENREALM_POINT_CLOUD_H
