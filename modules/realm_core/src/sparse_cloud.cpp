#include <realm_core/sparse_cloud.h>

using namespace realm;

SparseCloud::SparseCloud()
{
}

SparseCloud::SparseCloud(const std::vector<uint32_t> &point_ids, const cv::Mat &data)
 :  m_point_ids(point_ids),
    m_data(data)
{
  if (data.rows != m_point_ids.size())
    throw(std::invalid_argument("Error creating sparse cloud: Data - ID mismatch!"));
}

bool SparseCloud::empty()
{
  if (m_data.rows == 0)
    return true;
  return false;
}

cv::Mat& SparseCloud::data()
{
  return m_data;
}

std::vector<uint32_t> SparseCloud::getPointIds()
{
  return m_point_ids;
}

int SparseCloud::size()
{
  return m_data.rows;
}