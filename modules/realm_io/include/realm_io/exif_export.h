

#ifndef PROJECT_EXIF_EXPORT_H
#define PROJECT_EXIF_EXPORT_H

#include <string>
#include <vector>

#include <exiv2/exiv2.hpp>

#include <realm_core/camera.h>
#include <realm_core/frame.h>
#include "gis_export.h"
#include <realm_io/utilities.h>
#include <realm_core/conversions.h>

namespace realm
{
namespace io
{

struct ExifMetaTag
{
    Exiv2::ExifData exif_data;
    Exiv2::XmpData xmp_data;
};

/*!
 * @brief Simplified interfaces to save a single frame at a destination directory.
 * @param frame Input frame that needs to be saved
 * @param directory Destination directory
 * @param name Name of the file
 * @param id Id of the saved file, usually the frame id
 * @param use_resized Flag to use resized frame elements based on the frame resize factor
 */
void saveExifImage(const Frame::Ptr &frame,
                   const std::string &directory,
                   const std::string &name,
                   uint32_t id,
                   bool use_resized);

/*!
 * @brief Saving function for image data with exif tags
 * @param img Image data
 * @param cam Camera data corresponding to the size of the image
 * @param utm_ref UTM position measurement
 * @param camera_id Unique camera id
 * @param image_id Unique frame id
 * @param filename Absolute filepath
 */
void saveExifImage(uint64_t timestamp,
                   const cv::Mat& img,
                   const camera::Pinhole::ConstPtr &cam,
                   const UTMPose &utm_ref,
                   const std::string &camera_id,
                   uint32_t image_id,
                   const std::string &filename);

/*!
 * @brief Function to convert an input wgs84 coordinate to exif saveable format
 * @param wgs Wgs84 coordinate with lat/lon/alt
 * @return vector with string formatted as:
 * vect[0]: GNSS Version ID
 * vect[1]: Above Sea Level ? 0 : 1
 * vect[2]: Altitude/1
 * vect[3]: Above Equator ? N : S
 * vect[4]: Latitude
 * vect[5]: East of green meridian ? E : W
 * vect[6]: Longitude
 */
std::vector<std::string> createGNSSExifTag(const WGSPose &wgs);


/*!
 * @brief Conversion of decimal angle to degree, minute, second format
 * @param angle Angle in degree
 * @return formatted string for exif saving of GNSS
 */
std::string cvtAngleDecimalToDegMinSec(double angle);

}
} // namespace realm

#endif //PROJECT_EXIF_EXPORT_H
