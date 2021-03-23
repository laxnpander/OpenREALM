#ifndef OPENREALM_SPARSE_CLOUD_H
#define OPENREALM_SPARSE_CLOUD_H

#include <opencv2/core.hpp>
#include <memory>

namespace realm
{

class SparseCloud
{
public:
  using Ptr = std::shared_ptr<SparseCloud>;

public:
  SparseCloud();
  SparseCloud(const std::vector<uint32_t> &point_ids, const cv::Mat &data);

  bool empty();

  cv::Mat& data();

  int size();

  std::vector<int> getPointIds();

private:
  std::vector<uint32_t> m_point_ids;
  cv::Mat m_data;

};

} // namespace realm

#endif //OPENREALM_SPARSE_CLOUD_H
