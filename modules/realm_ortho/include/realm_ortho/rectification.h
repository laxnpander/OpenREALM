

#ifndef PROJECT_RECTIFICATION_H
#define PROJECT_RECTIFICATION_H

#include <memory>
#include <string>
#include <cmath>

#include <realm_core/structs.h>
#include <realm_core/frame.h>
#include <realm_core/cv_grid_map.h>

namespace realm
{
namespace ortho
{

/*!
 * @brief Simplified interface function for rectification
 * @param frame container for aerial measurement data
 * @return Rectified input data
 */
CvGridMap::Ptr rectify(const Frame::Ptr &frame);

/*!
 * @brief Rectification is achieved using the workflow presented in: http://www.timohinzmann.com/publications/fsr_2017_hinzmann.pdf.
 * It is assumed, that we have a 2.5D surface grid with an elevation value in each cell. The coordinates of the cell in turn
 * resemble a geographic point with x = utm eastings, y = utm northing. Then the camera model is applied to backproject every
 * world point into the camera image. The math does not change, when the surface is planar (elevation = 0 for each cell).
 * @param img Image data that is corrected from lens distortion
 * @param cam Underlying camera model, currently only pinhole camera is supported
 * @param surface Surface structure as matrix with each element resembling the elevation to a reference plane
 * @param roi Region of interest in geographic coordinates with (x, y) = lower left corner
 * @param GSD Ground sampling distance, therefore the resolution of the surface cells
 * @param is_elevated Flag to set whether the surface is planar or has elevation
 */
CvGridMap::Ptr backprojectFromGrid(
    const cv::Mat &img,
    const camera::Pinhole &cam,
    cv::Mat &surface,
    const cv::Rect2d &roi,
    double GSD,
    bool is_elevated,
    bool verbose = true);

namespace internal
{

inline double computeElevationAngle(double t[3], double p[3]);

} // namespace internal
} // namespace ortho
} // namespace realm

#endif //PROJECT_RECTIFICATION_H
