

#ifndef PROJECT_REALM_EXPORT_H
#define PROJECT_REALM_EXPORT_H

#include <fstream>

#include <eigen3/Eigen/Eigen>

#include <realm_core/cv_grid_map.h>
#include <realm_core/camera.h>
#include <realm_io/utilities.h>

namespace realm
{
namespace io
{

void saveTimestamp(uint64_t timestamp,
                   uint32_t frame_id,
                   const std::string &filename);

void saveTrajectory(uint64_t timestamp,
                    const cv::Mat &pose,
                    const std::string &filepath);

void saveTrajectoryTUM(std::ofstream *file,
                       uint64_t timestamp,
                       double x,
                       double y,
                       double z,
                       double qx,
                       double qy,
                       double qz,
                       double qw);

/*!
 * @brief Saving a CvGridMap as binary. Data is uncompressed and can therefore be of any opencv type (CV_8UC4, CV_32F, ...).
 * For compressed data image encoding is required, which does not allow to save e.g. floats.
 * @param map CvGridMap about to be saved to disk
 * @param filepath Absolute filepath with .grid.bin suffix
 */
void saveCvGridMap(const CvGridMap &map, const std::string &filepath);

} // namespace io
} // namespace realm

#endif //PROJECT_REALM_EXPORT_H
