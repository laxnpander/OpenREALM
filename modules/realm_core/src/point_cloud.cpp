#include <realm_core/point_cloud.h>

using namespace realm;

PointCloud::PointCloud()
{
}

PointCloud::PointCloud(const std::vector<uint32_t> &point_ids, const cv::Mat &data)
 :  m_point_ids(point_ids),
    m_data(data)
{
  if (data.rows != m_point_ids.size())
    throw(std::invalid_argument("Error creating sparse cloud: Data - ID mismatch!"));
}

bool PointCloud::empty()
{
  if (m_data.empty() || m_data.rows == 0)
    return true;
  return false;
}

cv::Mat& PointCloud::data()
{
  return m_data;
}

std::vector<uint32_t> PointCloud::getPointIds()
{
  return m_point_ids;
}

int PointCloud::size()
{
  return m_data.rows;
}