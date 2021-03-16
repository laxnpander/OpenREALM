

#ifndef PROJECT_ANALYSIS_H
#define PROJECT_ANALYSIS_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace realm
{
namespace analysis
{

/*!
 * @brief Converts a single channel floating point mat to a RGB color map
 * @param img CV_32FC1 or CV_64FC1 floating point mat
 * @param mask Mask of valid pixels
 * @param flag Color layout
 * @return Colormap of input mat
 */
cv::Mat convertToColorMapFromCVC1(const cv::Mat &img, const cv::Mat &mask, cv::ColormapTypes flag);

/*!
 * @brief Converts a three channel floating point mat to a RGB color map
 * @param img CV_32FC3 or CV_64FC3 floating point mat
 * @param mask Mask of valid pixels
 * @return Colormap of input mat
 */
cv::Mat convertToColorMapFromCVC3(const cv::Mat &img, const cv::Mat &mask);

} // namespace analysis
} // namespace realm

#endif //PROJECT_ANALYSIS_H
