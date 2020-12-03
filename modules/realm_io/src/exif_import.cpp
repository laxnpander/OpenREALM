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

#include <realm_core/loguru.h>

namespace realm
{

io::Exiv2FrameReader::Exiv2FrameReader(const FrameTags &tags)
: _frame_tags(tags)
{
}

std::map<std::string, bool> io::Exiv2FrameReader::probeImage(const std::string &filepath)
{
  std::map<std::string, bool> tag_existence;
  tag_existence[_frame_tags.camera_id] = false;
  tag_existence[_frame_tags.timestamp] = false;
  tag_existence[_frame_tags.latitude] = false;
  tag_existence[_frame_tags.longitude] = false;
  tag_existence[_frame_tags.altitude] = false;
  tag_existence[_frame_tags.heading] = false;

  Exiv2ImagePointer exif_img = Exiv2::ImageFactory::open(filepath);
  if (exif_img.get())
  {
    exif_img->readMetadata();
    Exiv2::ExifData &exif_data = exif_img->exifData();
    Exiv2::XmpData &xmp_data = exif_img->xmpData();

    tag_existence[_frame_tags.camera_id] = probeTag(_frame_tags.camera_id, exif_data, xmp_data);
    tag_existence[_frame_tags.timestamp] = probeTag(_frame_tags.timestamp, exif_data, xmp_data);
    tag_existence[_frame_tags.latitude] = probeTag(_frame_tags.latitude, exif_data, xmp_data);
    tag_existence[_frame_tags.longitude] = probeTag(_frame_tags.longitude, exif_data, xmp_data);
    tag_existence[_frame_tags.altitude] = probeTag(_frame_tags.altitude, exif_data, xmp_data);
    tag_existence[_frame_tags.heading] = probeTag(_frame_tags.heading, exif_data, xmp_data);
  }

  return tag_existence;
}

Frame::Ptr io::Exiv2FrameReader::loadFrameFromExiv2(const std::string &camera_id, const camera::Pinhole::Ptr &cam, const std::string &filepath)
{
  Exiv2ImagePointer exif_img = Exiv2::ImageFactory::open(filepath);
  if (exif_img.get())
  {
    // Read exif and xmp metadata
    exif_img->readMetadata();
    Exiv2::ExifData &exif_data = exif_img->exifData();
    Exiv2::XmpData &xmp_data = exif_img->xmpData();

    // Read image data
    cv::Mat img = cv::imread(filepath, cv::IMREAD_COLOR);

    /*========== ESSENTIAL KEYS ==========*/
    uint32_t frame_id = io::extractFrameIdFromFilepath(filepath);

    std::string camera_id_set;
    if (camera_id.empty())
    {
      if (!readMetaTagCameraId(exif_data, xmp_data, &camera_id_set))
        camera_id_set = "unknown_id";
    }

    WGSPose wgs{0};
    if (!readMetaTagLatitude(exif_data, xmp_data, &wgs.latitude))
      wgs.latitude = 0.0;

    if (!readMetaTagLongitude(exif_data, xmp_data, &wgs.longitude))
      wgs.longitude = 0.0;

    if (!readMetaTagAltitude(exif_data, xmp_data, &wgs.altitude))
      wgs.altitude = 0.0;

    if (!readMetaTagHeading(exif_data, xmp_data, &wgs.heading))
      wgs.heading = 0.0;

    UTMPose utm = gis::convertToUTM(wgs);

    //LOG_F(INFO, "WGS: %f %f", wgs.latitude, wgs.longitude);
    //LOG_F(INFO, "UTM: %d %c %f %f", utm.zone, utm.band, utm.easting, utm.northing);

    /*========== OPTIONAL KEYS ==========*/
    uint64_t timestamp_val;
    if (!readMetaTagTimestamp(exif_data, xmp_data, &timestamp_val))
      timestamp_val = getCurrentTimeNano();

    return std::make_shared<Frame>(camera_id, frame_id, timestamp_val, img, utm, cam, computeOrientationFromHeading(utm.heading));
  }
  return nullptr;
}

bool io::Exiv2FrameReader::readMetaTagCameraId(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, std::string* camera_id)
{
  if (isXmpTag(_frame_tags.camera_id))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(_frame_tags.camera_id)) != xmp_data.end())
    {
      *camera_id = xmp_data[_frame_tags.camera_id].toString();
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(_frame_tags.camera_id)) != exif_data.end())
    {
      *camera_id = exif_data[_frame_tags.camera_id].toString();
      return true;
    }
  }
  return false;
}

bool io::Exiv2FrameReader::readMetaTagTimestamp(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, uint64_t* timestamp)
{
  if (isXmpTag(_frame_tags.timestamp))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(_frame_tags.timestamp)) != xmp_data.end())
    {
      *timestamp = std::stoul(xmp_data[_frame_tags.timestamp].toString());
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(_frame_tags.timestamp)) != exif_data.end())
    {
      *timestamp = std::stoul(exif_data[_frame_tags.timestamp].toString());
      return true;
    }
  }
  return false;
}

bool io::Exiv2FrameReader::readMetaTagLatitude(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* latitude)
{
  double latitude_dms[3];
  if (isXmpTag(_frame_tags.latitude))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(_frame_tags.latitude)) != xmp_data.end())
    {
      latitude_dms[0] = xmp_data[_frame_tags.latitude].toFloat(0);
      latitude_dms[1] = xmp_data[_frame_tags.latitude].toFloat(1);
      latitude_dms[2] = xmp_data[_frame_tags.latitude].toFloat(2);
      *latitude = cvtAngleDegMinSecToDecimal(latitude_dms);
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(_frame_tags.latitude)) != exif_data.end())
    {
      latitude_dms[0] = exif_data[_frame_tags.latitude].toFloat(0);
      latitude_dms[1] = exif_data[_frame_tags.latitude].toFloat(1);
      latitude_dms[2] = exif_data[_frame_tags.latitude].toFloat(2);
      *latitude = cvtAngleDegMinSecToDecimal(latitude_dms);
      return true;
    }
  }
  return false;
}

bool io::Exiv2FrameReader::readMetaTagLongitude(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* longitude)
{
  double longitude_dms[3];
  if (isXmpTag(_frame_tags.longitude))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(_frame_tags.longitude)) != xmp_data.end())
    {
      longitude_dms[0] = xmp_data[_frame_tags.longitude].toFloat(0);
      longitude_dms[1] = xmp_data[_frame_tags.longitude].toFloat(1);
      longitude_dms[2] = xmp_data[_frame_tags.longitude].toFloat(2);
      *longitude = cvtAngleDegMinSecToDecimal(longitude_dms);
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(_frame_tags.longitude)) != exif_data.end())
    {
      longitude_dms[0] = exif_data[_frame_tags.longitude].toFloat(0);
      longitude_dms[1] = exif_data[_frame_tags.longitude].toFloat(1);
      longitude_dms[2] = exif_data[_frame_tags.longitude].toFloat(2);
      *longitude = cvtAngleDegMinSecToDecimal(longitude_dms);
      return true;
    }
  }
  return false;
}

bool io::Exiv2FrameReader::readMetaTagAltitude(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* altitude)
{
  if (isXmpTag(_frame_tags.latitude))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(_frame_tags.latitude)) != xmp_data.end())
    {
      *altitude = static_cast<double>(xmp_data[_frame_tags.altitude].toFloat());
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(_frame_tags.latitude)) != exif_data.end())
    {
      *altitude = static_cast<double>(exif_data[_frame_tags.altitude].toFloat());
      return true;
    }
  }
  return false;
}

bool io::Exiv2FrameReader::readMetaTagHeading(Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data, double* heading)
{
  if (isXmpTag(_frame_tags.heading))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(_frame_tags.heading)) != xmp_data.end())
    {
      *heading = static_cast<double>(xmp_data[_frame_tags.heading].toFloat());
      return true;
    }
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(_frame_tags.heading)) != exif_data.end())
    {
      *heading = static_cast<double>(exif_data[_frame_tags.heading].toFloat());
      return true;
    }
  }
  return false;
}

double io::Exiv2FrameReader::cvtAngleDegMinSecToDecimal(const double* angle)
{
  double angle_deg = angle[0];
  double angle_min = angle[1]/60;
  double angle_sec = angle[2]/3600;
  return angle_deg + angle_min + angle_sec;
}

bool io::Exiv2FrameReader::isXmpTag(const std::string &tag)
{
  std::vector<std::string> tokens = io::split(tag.c_str(), '.');
  return tokens[0] == "Xmp";
}

bool io::Exiv2FrameReader::probeTag(const std::string &tag, Exiv2::ExifData &exif_data, Exiv2::XmpData &xmp_data)
{
  if (isXmpTag(tag))
  {
    if (xmp_data.findKey(Exiv2::XmpKey(tag)) != xmp_data.end())
      return true;
  }
  else
  {
    if (exif_data.findKey(Exiv2::ExifKey(tag)) != exif_data.end())
      return true;
  }
  return false;
}

}