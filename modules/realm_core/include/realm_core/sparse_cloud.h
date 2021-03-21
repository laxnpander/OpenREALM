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
  SparseCloud(uint32_t context_id, const std::vector<uint32_t> &point_ids, const cv::Mat &data);

  bool empty();

  cv::Mat& data();

  int size();

  uint32_t getContextId();
  std::vector<int> getPointIds();

private:
  uint32_t m_context_id;
  std::vector<uint32_t> m_point_ids;
  cv::Mat m_data;

};

} // namespace realm

#endif //OPENREALM_SPARSE_CLOUD_H
