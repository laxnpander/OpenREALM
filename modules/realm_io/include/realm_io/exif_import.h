/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PROJECT_EXIF_IMPORT_H
#define PROJECT_EXIF_IMPORT_H

#include <exiv2/exiv2.hpp>

#include <realm_core/conversions.h>
#include <realm_core/frame.h>
#include <realm_core/wgs84.h>
#include "gis_export.h"
#include <realm_io/utilities.h>

namespace realm
{
namespace io
{

// Exiv2 changed the image pointer definition in version 0.27 and higher
#if EXIV2_MINOR_VERSION >= 27
  using Exiv2ImagePointer = Exiv2::Image::UniquePtr;
#else
  using Exiv2ImagePointer = Exiv2::Image::AutoPtr;
#endif

class Exiv2FrameReader
{
  public:
    struct FrameTags
    {
        FrameTags()
        : timestamp("Xmp.exif.REALM.Timestamp"),
          camera_id("Exif.Image.Model"),
          heading("Xmp.exif.REALM.Heading"),
          latitude("Exif.GPSInfo.GPSLatitude"),
          longitude("Exif.GPSInfo.GPSLongitude"),
          altitude("Exif.GPSInfo.GPSAltitude")
        {
        }

        FrameTags(const std::string &timestamp,
                  const std::string &camera_id,
                  const std::string &heading,
                  const std::string &latitude,
                  const std::string &longitude,
                  const std::string &altitude)
        : timestamp(timestamp),
          camera_id(camera_id),
          heading(heading),
          latitude(latitude),
          longitude(longitude),
          altitude(altitude)
        {
        }

        static FrameTags loadFromFile(const std::string &filepath)
        {
          cv::FileStorage fs(filepath, cv::FileStorage::READ);
          try
          {
            return FrameTags
                    {
                      fs["Frame.timestamp"],
                      fs["Frame.camera_id"],
                      fs["Frame.heading"],
                      fs["Frame.latitude_tag"],
                      fs["Frame.longitude_tag"],
                      fs["Frame.altitude_tag"]
                    };
          }
          catch(...)
          {

          }
        }

        std::string timestamp;
        std::string camera_id;
        std::string heading;
        std::string latitude;
        std::string longitude;
        std::string altitude;
    };

  public:
    Exiv2FrameReader() = default;

    /*!
     * @brief Constructor for Exiv2 frame reader initialized from self defined frame tags
     * @param tags Tags to be read for frame information
     */
    explicit Exiv2FrameReader(const FrameTags &tags);

    /*!
     * @brief Function for loading REALM frame from image with Exif tags. Note that camera informations currently must be
     *        provided additionally, to keep usability for different sources (e.g. DJI, REALM, ...)
     * @param cam Additional camera informations, currently only pinhole supported
     * @param filepath Path to the image with exif tags
     * @return Basic Frame for REALM framework, Pose must be set outside
     */
     Frame::Ptr loadFrameFromExiv2(const std::string &camera_id, const camera::Pinhole::Ptr &cam, const std::string &filepath);

    /*!
     * @brief Reading GNSS informations from exif tags. There are several formats out there, main format is lat/lon in
     *        degree, minute and second. Must therefore be converted. GNSS informations are provided according to exiv2
     *        standard, see: http://www.exiv2.org/tags.html
     * @param exif_data Exif data struct as key/value pairs
     * @return Pose in WGS84 coordinates
     */
     WGSPose readGNSSExifTag(Exiv2::ExifData &exif_data);

  private:

    //! Tags to be read from the exiv information
    FrameTags _frame_tags;

    /*!
     * @brief Conversion function for angle in degree, minute, second format to decimal angle
     * @param angle Array of 3 elements (double[3]) with
     *              angle[0]: degree
     *              angle[1]: minute
     *              angle[2]: second
     * @return Angle in decimals
     */
    double cvtAngleDegMinSecToDecimal(const double* angle);
};

} // namespace io
} // namespace realm

#endif //PROJECT_EXIF_IMPORT_H
