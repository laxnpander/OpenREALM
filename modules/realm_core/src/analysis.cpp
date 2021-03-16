

#include <opencv2/imgproc/imgproc_c.h>

#include <realm_core/analysis.h>
#include <realm_core/loguru.h>

using namespace realm;

cv::Mat analysis::convertToColorMapFromCVC1(const cv::Mat &img, const cv::Mat &mask, cv::ColormapTypes flag)
{
  assert(img.type() == CV_32FC1 || img.type() == CV_64FC1 || img.type() == CV_16UC1);

  cv::Mat map_norm;
  cv::Mat map_colored;

  // Normalization can be processed with mask or without
  if (mask.empty())
    cv::normalize(img, map_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  else
    cv::normalize(img, map_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1, mask);

  // Afterwards apply color map
  cv::applyColorMap(map_norm, map_colored, flag);

  // Set invalid pixels black
  for (int r = 0; r < map_colored.rows; ++r)
    for (int c = 0; c < map_colored.cols; ++c)
      if (mask.at<uchar>(r, c) == 0)
        map_colored.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);

  return map_colored;
}

cv::Mat analysis::convertToColorMapFromCVC3(const cv::Mat &img, const cv::Mat &mask)
{
  assert(img.type() == CV_32FC3 || img.type() == CV_64FC3 || img.type() == CV_16UC3);

  cv::Mat map_32fc3, map_8uc3;
  cv::cvtColor(img, map_32fc3, CV_XYZ2BGR);
  map_32fc3 *= 255;
  map_32fc3.convertTo(map_8uc3, CV_8UC3);

  // Set invalid pixels black
  for (int r = 0; r < map_8uc3.rows; ++r)
    for (int c = 0; c < map_8uc3.cols; ++c)
      if (mask.at<uchar>(r, c) == 0)
        map_8uc3.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);

  return map_8uc3;
}