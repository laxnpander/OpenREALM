

#include <realm_core/depthmap.h>

using namespace realm;

Depthmap::Depthmap(const cv::Mat &data, const camera::Pinhole &cam)
 : m_data(data),
   m_cam(std::make_shared<camera::Pinhole>(cam))
{
  if (m_data.type() != CV_32F)
    throw(std::invalid_argument("Error creating depth map: Matrix type not CV_32F"));
  if (data.rows != m_cam->height() || data.cols != m_cam->width())
    throw(std::invalid_argument("Error creating depth map: Dimension mismatch! Camera size does not match data."));

  updateDepthParameters();
}

camera::Pinhole::ConstPtr Depthmap::getCamera() const
{
  return m_cam;
}

double Depthmap::getMinDepth() const
{
  return m_min_depth;
}

double Depthmap::getMaxDepth() const
{
  return m_max_depth;
}

double Depthmap::getMedianDepth() const
{
  return m_med_depth;
}

cv::Mat& Depthmap::data()
{
  return m_data;
}

void Depthmap::updateDepthParameters()
{
  // First grab min and max values
  cv::Mat mask = (m_data > 0);
  cv::minMaxLoc(m_data, &m_min_depth, &m_max_depth, nullptr, nullptr, mask);

  std::vector<float> array;
  if (m_data.isContinuous()) {
    array.assign((float*)m_data.data, (float*)m_data.data + m_data.total() * m_data.channels());
  } else {
    for (int i = 0; i < m_data.rows; ++i) {
      array.insert(array.end(), m_data.ptr<float>(i), m_data.ptr<float>(i) + m_data.cols * m_data.channels());
    }
  }

  // Invalid depth values are set to -1.0
  std::vector<float> array_masked;
  std::copy_if(array.begin(), array.end(), back_inserter(array_masked),[](float n){ return  n > 0.0;});

  // Find median by sorting the array and get middle value
  std::sort(array.begin(), array.end());
  m_med_depth = array[array.size() / 2];
}