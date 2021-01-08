

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

// Exiv2 changed the image pointer definition in master, maybe unreleased version 0.28 and higher
#if EXIV2_MINOR_VERSION >= 28
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
          latituderef("Exif.GPSInfo.GPSLatitudeRef"),
          longitude("Exif.GPSInfo.GPSLongitude"),
          longituderef("Exif.GPSInfo.GPSLongitudeRef"),
          altitude("Exif.GPSInfo.GPSAltitude")
        {
        }

        FrameTags(const std::string &timestamp,
                  const std::string &camera_id,
                  const std::string &heading,
                  const std::string &latitude,
                  const std::string &latituderef,
                  const std::string &longitude,
                  const std::string &longituderef,
                  const std::string &altitude)
        : timestamp(timestamp),
          camera_id(camera_id),
          heading(heading),
          latitude(latitude),
          latituderef(latituderef),
          longitude(longitude),
          longituderef(longituderef),
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
                      fs["Frame.latituderef_tag"],
                      fs["Frame.longitude_tag"],
                      fs["Frame.longituderef_tag"],
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
        std::string latituderef;
        std::string longitude;
        std::string longituderef;
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
   * @brief Loads an image's meta data and checks if it finds all the required tags.
   * @param filepath Absolute path to the image that is being probed
   * @return Key/value pairs of 'tag name' and wether it was found in the image or not
   */
  std::map<std::string, bool> probeImage(const std::string &filepath);

  /*!
   * @brief Function for loading REALM frame from image with Exif tags. Note that camera informations currently must be
   *        provided additionally, to keep usability for different sources (e.g. DJI, REALM, ...)
   * @param cam Additional camera informations, currently only pinhole supported
   * @param filepath Path to the image with exif tags
   * @return Basic Frame for REALM framework, Pose must be set outside
   */
  Frame::Ptr loadFrameFromExiv2(const std::string &camera_id, const camera::Pinhole::Ptr &cam, const std::string &filepath);

private:

  //! Tags to be read from the exiv information
  FrameTags m_frame_tags;

  /*!
   * @brief Conversion function for angle in degree, minute, second format to decimal angle
   * @param angle Array of 3 elements (double[3]) with
   *              angle[0]: degree
   *              angle[1]: minute
   *              angle[2]: second
   * @return Angle in decimals
   */
  double cvtAngleDegMinSecToDecimal(const double *angle);

  /*!
   * @brief Helper function that splits the tag name at '.' and checks if the first token is equal to 'Xmp' to identify
   * wether the tag is an xmp tag or an exif tag.
   * @param tag Name of the tag
   * @return true if tag is an xmp tag
   */
  bool isXmpTag(const std::string &tag);

  /*!
   * @brief Searches tags for specific tag and returns true if was found in either Exif or Xmp data.
   * @param tag Name of the tag
   * @param exif_data Collection of exif tags
   * @param xmp_data Collection of xmp tags
   * @return true if found
   */
  bool probeTag(const std::string &tag, Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data);

  /*!
   * @brief Searches the meta tags for the camera id and returns true if found. The value is provided in the parameter list.
   * @param exif_data Collection of exif tags
   * @param xmp_data Collection of xmp tags
   * @param camera_id Output camera id
   * @return True if tag was found
   */
  bool readMetaTagCameraId(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, std::string* camera_id);

  /*!
   * @brief Searches the meta tags for the timestamp tag and returns true if found. The value is provided in the parameter list.
   * @param exif_data Collection of exif tags
   * @param xmp_data Collection of xmp tags
   * @param camera_id Output timestamp
   * @return True if tag was found
   */
  bool readMetaTagTimestamp(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, uint64_t* timestamp);

  /*!
   * @brief Searches the meta tags for the GNSS latitude and returns true if found. The value is provided in the parameter list.
   * @param exif_data Collection of exif tags
   * @param xmp_data Collection of xmp tags
   * @param camera_id Output latitude
   * @return True if tag was found
   */
  bool readMetaTagLatitude(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* latitude);

  /*!
   * @brief Searches the meta tags for the GNSS longitude and returns true if found. The value is provided in the parameter list.
   * @param exif_data Collection of exif tags
   * @param xmp_data Collection of xmp tags
   * @param camera_id Output longitude
   * @return True if tag was found
   */
  bool readMetaTagLongitude(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* longitude);

  /*!
   * @brief Searches the meta tags for the relative altitude and returns true if found. The value is provided in the parameter list.
   * @param exif_data Collection of exif tags
   * @param xmp_data Collection of xmp tags
   * @param camera_id Output altitude
   * @return True if tag was found
   */
  bool readMetaTagAltitude(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* altitude);

  /*!
   * @brief Searches the meta tags for the magnetic heading and returns true if found. The value is provided in the parameter list.
   * @param exif_data Collection of exif tags
   * @param xmp_data Collection of xmp tags
   * @param camera_id Output heading
   * @return True if tag was found
   */
  bool readMetaTagHeading(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* heading);
};

} // namespace io
} // namespace realm

#endif //PROJECT_EXIF_IMPORT_H
