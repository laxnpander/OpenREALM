

#include <realm_vslam_base/dummy_referencer.h>

#include <realm_core/loguru.h>

using namespace realm;

DummyReferencer::DummyReferencer(const cv::Mat &T_c2g)
  : m_transformation_c2g(T_c2g)
{
}

void DummyReferencer::init(const std::vector<Frame::Ptr> &frames)
{
  LOG_F(WARNING, "This is a dummy georeferenciation. You have to manually provide the transformation from camera to world frame. "
                 "Not call to 'init()' possible!");
}

cv::Mat DummyReferencer::getTransformation()
{
  std::unique_lock<std::mutex> lock(m_mutex_t_c2g);
  return m_transformation_c2g;
}

void DummyReferencer::update(const Frame::Ptr &frame)
{
}

bool DummyReferencer::isBuisy()
{
  return false;
}

bool DummyReferencer::isInitialized()
{
  return true;
}