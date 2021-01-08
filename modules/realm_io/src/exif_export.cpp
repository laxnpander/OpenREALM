

#include <realm_io/exif_export.h>

namespace realm
{
namespace io
{

void saveExifImage(const Frame::Ptr &frame,
                   const std::string &directory,
                   const std::string &name,
                   uint32_t id,
                   bool use_resized)
{
  std::string filename = io::createFilename(directory + "/" + name + "_", id, ".jpg");
  if (use_resized)
    saveExifImage(frame->getTimestamp(),
                  frame->getResizedImageRaw(),
                  frame->getResizedCamera(),
                  frame->getGnssUtm(),
                  frame->getCameraId(),
                  frame->getFrameId(),
                  filename);
  else
    saveExifImage(frame->getTimestamp(),
                  frame->getImageRaw(),
                  frame->getCamera(),
                  frame->getGnssUtm(),
                  frame->getCameraId(),
                  frame->getFrameId(),
                  filename);
}

void saveExifImage(uint64_t timestamp,
                   const cv::Mat& img,
                   const camera::Pinhole::ConstPtr &cam,
                   const UTMPose &utm_ref,
                   const std::string &camera_id,
                   uint32_t image_id,
                   const std::string &filename)
{
  ExifMetaTag meta;

  // General Tags
  meta.exif_data["Exif.Image.ProcessingSoftware"] = "REALM";
  meta.exif_data["Exif.Image.Model"]              = camera_id;
  meta.exif_data["Exif.Photo.DateTimeOriginal"]   = getDateTime();
  meta.exif_data["Exif.Image.ImageNumber"]        = image_id;            // Long

  // GNSS info conversion
  std::vector<std::string> gnss_info = createGNSSExifTag(gis::convertToWGS84(utm_ref));
  meta.exif_data["Exif.GPSInfo.GPSVersionID"]      = gnss_info[0];            // Byte
  meta.exif_data["Exif.GPSInfo.GPSLatitudeRef"]    = gnss_info[3];            // Ascii
  meta.exif_data["Exif.GPSInfo.GPSLatitude"]       = gnss_info[4];            // Rational
  meta.exif_data["Exif.GPSInfo.GPSLongitudeRef"]   = gnss_info[5];            // Ascii
  meta.exif_data["Exif.GPSInfo.GPSLongitude"]      = gnss_info[6];            // Rational
  meta.exif_data["Exif.GPSInfo.GPSAltitudeRef"]    = gnss_info[1];            // Byte
  meta.exif_data["Exif.GPSInfo.GPSAltitude"]       = gnss_info[2];            // Rational

  // Camera calibration tags
  meta.xmp_data["Xmp.exif.Timestamp"] = timestamp;
  meta.xmp_data["Xmp.exif.cx"] = cam->cx();
  meta.xmp_data["Xmp.exif.cy"] = cam->cy();
  meta.xmp_data["Xmp.exif.fx"] = cam->fx();
  meta.xmp_data["Xmp.exif.fy"] = cam->fy();

  // Writing image data from opencv mat is not straightforward, see http://dev.exiv2.org/boards/3/topics/2795 for info
  // Basic idea: encode img data before passing to exif creation
  std::vector<uchar> buffer;
  cv::imencode(".jpg", img, buffer);

  std::unique_ptr<Exiv2::Image> exiv2_file = Exiv2::ImageFactory::open(&buffer[0], buffer.size());

  if (exiv2_file != nullptr)
  {
    exiv2_file->setExifData(meta.exif_data);
    exiv2_file->setXmpData(meta.xmp_data);
    exiv2_file->writeMetadata();

    // Workaround for writing exif tags to image. File must be created before opening with exif API
    FILE * file1 = ::fopen(filename.c_str(),"w");
    if (file1 != nullptr)
      ::fclose(file1);
    else
      throw(std::runtime_error("Error creating exif file: Opening '" + filename + "' failed!"));

    Exiv2::FileIo file(filename);
    if (!file.open())
    {
      file.write(exiv2_file->io());
      file.close();
    }
    else
      throw(std::runtime_error("Error creating exif file: Opening '" + filename + "' failed!"));
  }
  else
    throw(std::runtime_error("Error creating exif image data: Opening failed!"));
}

std::vector<std::string> createGNSSExifTag(const WGSPose &wgs)
{
  std::vector<std::string> vect(7);
  // GNSS Version ID
  vect[0] = "2 2 2 2";

  // Altitude
  if (wgs.altitude >= 0.0 )
    vect[1] = "0";      // Above Sea Level
  else
    vect[1] = "1";
  vect[2] =  std::to_string((int)floor(fabs(wgs.altitude))) + "/1";

  // Latitude
  if (wgs.latitude >= 0.0 )
    vect[3] = "N";  // Above Equator
  else
    vect[3] = "S";
  vect[4] = cvtAngleDecimalToDegMinSec(wgs.latitude);

  // Longitude
  if (wgs.longitude >= 0.0 )
    vect[5] = "E";     // East of green meridian
  else
    vect[5] = "W";
  vect[6] = cvtAngleDecimalToDegMinSec(wgs.longitude);
  return vect;
}

std::string cvtAngleDecimalToDegMinSec(double angle)
{
  std::stringstream result;
  double angle_abs = fabs(angle);
  auto angle_deg = static_cast<int>(floor(angle_abs));
  double angle_rem = (angle_abs - angle_deg)*60;
  auto angle_min = static_cast<int>(floor(angle_rem));
  auto angle_sec = static_cast<int>(floor((angle_rem - angle_min)*6000));
  result << angle_deg << "/1 " << angle_min << "/1 " << angle_sec << "/100";
  return result.str();
}

} // namespace io
} // namespace realm