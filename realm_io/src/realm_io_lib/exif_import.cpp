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

#include <realm_io/exif_import.h>

namespace realm
{

io::Exiv2FrameReader::Exiv2FrameReader(const FrameTags &tags)
: _frame_tags(tags)
{
}

Frame::Ptr io::Exiv2FrameReader::loadFrameFromExiv2(const std::string &camera_id, const camera::Pinhole &cam, const std::string &filepath)
{
  Exiv2::Image::UniquePtr exif_img = Exiv2::ImageFactory::open(filepath);
  if (exif_img.get())
  {
    // Read exif and xmp metadata
    exif_img->readMetadata();
    Exiv2::ExifData &exif_data = exif_img->exifData();
    Exiv2::XmpData &xmp_data = exif_img->xmpData();
    if (exif_data.empty())
      throw(std::invalid_argument("Error loading Exiv2 frame from filepath '" + filepath + "'."));

    // Read image data
    cv::Mat img = cv::imread(filepath, cv::IMREAD_COLOR);

    /*========== MUST HAVE KEYS ==========*/
    uint32_t frame_id = io::extractFrameIdFromFilepath(filepath);
    std::string camera_id_set;
    if (camera_id == "")
      camera_id_set = exif_data[_frame_tags.camera_id].toString();
    else
      camera_id_set = camera_id;

    WGSPose wgs = readGNSSExifTag(exif_data);

    if (xmp_data.findKey(Exiv2::XmpKey(_frame_tags.heading)) != xmp_data.end())
      wgs.heading = xmp_data[_frame_tags.heading].toFloat();

    UTMPose utm = gis::convertToUTM(wgs);

    /*========== OPTIONAL KEYS ==========*/
    uint64_t timestamp_val;
    if (xmp_data.findKey(Exiv2::XmpKey(_frame_tags.timestamp)) != xmp_data.end())
      timestamp_val = std::stoul(xmp_data[_frame_tags.timestamp].toString());
    else
      timestamp_val = getCurrentTimeNano();

    return std::make_shared<Frame>(camera_id, frame_id, timestamp_val, img, utm, cam);
  }
  return nullptr;
}

WGSPose io::Exiv2FrameReader::readGNSSExifTag(Exiv2::ExifData &exif_data)
{
  WGSPose wgs{0.0};
  wgs.altitude = exif_data[_frame_tags.altitude].toFloat();

  // Latitude in degree/minute/sec
  double latitude_dms[3];
  latitude_dms[0] = exif_data[_frame_tags.latitude].toFloat(0);
  latitude_dms[1] = exif_data[_frame_tags.latitude].toFloat(1);
  latitude_dms[2] = exif_data[_frame_tags.latitude].toFloat(2);
  wgs.latitude = cvtAngleDegMinSecToDecimal(latitude_dms);

  // Longitude
  double longitude_dms[3];
  longitude_dms[0] = exif_data[_frame_tags.longitude].toFloat(0);
  longitude_dms[1] = exif_data[_frame_tags.longitude].toFloat(1);
  longitude_dms[2] = exif_data[_frame_tags.longitude].toFloat(2);
  wgs.longitude = cvtAngleDegMinSecToDecimal(longitude_dms);

  return wgs;
}

double io::Exiv2FrameReader::cvtAngleDegMinSecToDecimal(const double* angle)
{
  double angle_deg = angle[0];
  double angle_min = angle[1]/60;
  double angle_sec = angle[2]/3600;
  return angle_deg + angle_min + angle_sec;
}


}