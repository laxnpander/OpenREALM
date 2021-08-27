

#include <realm_io/cv_export.h>

#include <fstream>

namespace realm
{
namespace io
{

void saveStereoPair(const Frame::Ptr &frame_left, const Frame::Ptr &frame_right, const std::string &path)
{
  // inner calib
  cv::Mat K_l = frame_left->getResizedCalibration();
  cv::Mat K_r = frame_right->getResizedCalibration();
  // distortion
  cv::Mat D_l = frame_left->getCamera()->distCoeffs();
  cv::Mat D_r = frame_right->getCamera()->distCoeffs();
  // exterior calib
  cv::Mat T_l_w2c = frame_left->getCamera()->Tw2c();
  cv::Mat T_r_c2w = frame_right->getCamera()->Tc2w();
  cv::Mat T_lr = T_l_w2c*T_r_c2w;
  // Now calculate transformation between the cameras
  // Formula: R = R_1^T * R_2
  //          t = R_1^T * t_2 - R_1^T*t_1
  cv::Mat R = T_lr.rowRange(0, 3).colRange(0, 3);
  cv::Mat t = T_lr.rowRange(0, 3).col(3);
  uint32_t width = frame_left->getResizedImageWidth();
  uint32_t height = frame_left->getResizedImageHeight();
  cv::Size img_size(width, height);
  // Rectify images by stereo parameters
  cv::Mat R1, R2;
  cv::Mat P1, P2;
  cv::Mat Q;
  cv::stereoRectify(K_l, D_l, K_r, D_r, img_size, R, t, R1, R2, P1, P2, Q);
  // create names
  char filename_img_left[1000];
  char filename_img_right[1000];
  char filename_intrinsic[1000];
  char filename_extrinsic[1000];
  sprintf(filename_img_left, (path + std::string("img_left_%06i.png")).c_str(), frame_right->getFrameId());
  sprintf(filename_img_right, (path + std::string("img_right_%06i.png")).c_str(), frame_right->getFrameId());
  sprintf(filename_intrinsic, (path + std::string("intrinsic_%06i.yml")).c_str(), frame_right->getFrameId());
  sprintf(filename_extrinsic, (path + std::string("extrinsic_%06i.yml")).c_str(), frame_right->getFrameId());
  // save image pair distorted
  if (cv::imwrite(std::string(filename_img_left), frame_left->getResizedImageUndistorted())
      && cv::imwrite(std::string(filename_img_right), frame_right->getResizedImageUndistorted()))
  {
    std::cout << "Saved stereo images to:\n" << filename_img_left << "\n" << filename_img_right << std::endl;
  }

  // save 1) intrisics and 2) extrinsics to folder
  cv::FileStorage fs(std::string(filename_intrinsic), cv::FileStorage::WRITE);
  if(fs.isOpened())
  {
    fs << "M1" << K_l << "D1" << D_l << "M2" << K_r << "D2" << D_r;
    std::cout << "Saved stereo intrinsics to:\n" << filename_intrinsic << std::endl;
    fs.release();
  }
  else
  {
    std::cout << "Error: can not save the intrinsic parameters to file:\n" << filename_intrinsic << std::endl;
  }
  fs.open(std::string(filename_extrinsic), cv::FileStorage::WRITE);
  if(fs.isOpened())
  {
    fs << "R" << R << "T" << t << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
    std::cout << "Saved stereo extrinsics to:\n" << filename_extrinsic << std::endl;
    fs.release();
  }
  else
  {
    std::cout << "Error: can not save the extrinsic parameters to file:\n" << filename_extrinsic << std::endl;
  }
}

void saveImage(const cv::Mat &img, const std::string &filename)
{
  std::string suffix = filename.substr(filename.size()-3, 3);

  if(suffix == "png" || suffix == "jpg")
    cv::imwrite(filename, img);
  else if (suffix == "bin")
    saveImageToBinary(img, filename);
  else
    throw(std::invalid_argument("Error writing image: Unknown suffix"));
}

void saveImageToBinary(const cv::Mat &data, const std::string &filepath)
{
  int elem_size_in_bytes = (int)data.elemSize();
  int elem_type          = (int)data.type();

  FILE* file = fopen((filepath).c_str(), "wb");

  int size[4] = {data.cols, data.rows, elem_size_in_bytes, elem_type};
  fwrite(size, 4, sizeof(int), file);

  // Operating rowise, so even non-continuous matrices are properly written to binary
  for (int r = 0; r < data.rows; ++r)
    fwrite(data.ptr<void>(r), data.cols, elem_size_in_bytes, file);

  fclose(file);
}

void saveDepthMap(const Depthmap::Ptr &depthmap, const std::string &filename, uint32_t id)
{
  cv::Mat data = depthmap->data();

  cv::Mat mask = (data > 0.0);

  cv::Mat img_normalized;
  if (data.type() == CV_32FC1 || data.type() == CV_64FC1)
    cv::normalize(data, img_normalized, 0, 65535, cv::NormTypes::NORM_MINMAX, CV_16UC1, mask);
  else
    throw(std::invalid_argument("Error saving depth map: Mat type not supported!"));

  char buffer[1000];
  sprintf(buffer, filename.c_str(), id);

  double min_depth, max_depth;
  cv::minMaxLoc(data, &min_depth, &max_depth, nullptr, nullptr, mask);

  std::string full_filename = std::string(buffer);
  std::ofstream metafile(full_filename.substr(0, full_filename.size()-3) + "txt");
  metafile << "Scaling\nMin: " << min_depth << "\nMax: " << max_depth;

  cv::Mat pose = depthmap->getCamera()->pose();
  metafile << "\nPose:";
  metafile << "\n" << pose.at<double>(0, 0) << " " << pose.at<double>(0, 1) << " " << pose.at<double>(0, 2) << " " << pose.at<double>(0, 3);
  metafile << "\n" << pose.at<double>(1, 0) << " " << pose.at<double>(1, 1) << " " << pose.at<double>(1, 2) << " " << pose.at<double>(1, 3);
  metafile << "\n" << pose.at<double>(2, 0) << " " << pose.at<double>(2, 1) << " " << pose.at<double>(2, 2) << " " << pose.at<double>(2, 3);
  metafile.close();

  cv::imwrite(full_filename, img_normalized);
}


void saveImageColorMap(const cv::Mat &img,
                       const cv::Mat &mask,
                       const std::string &directory,
                       const std::string &name,
                       ColormapType flag)
{
  std::string filename = (directory + "/" + name + ".png");
  saveImageColorMap(img, mask, filename, flag);
}

void saveImageColorMap(const cv::Mat &img,
                       const cv::Mat &mask,
                       const std::string &directory,
                       const std::string &name,
                       uint32_t frame_id,
                       ColormapType flag)
{
  std::string filename = io::createFilename(directory + "/" + name + "_", frame_id, ".png");
  saveImageColorMap(img, mask, filename, flag);
}

void saveImageColorMap(const cv::Mat &img,
                       float range_min,
                       float range_max,
                       const std::string &directory,
                       const std::string &name,
                       uint32_t frame_id,
                       ColormapType flag)
{
  std::string filename = io::createFilename(directory + "/" + name + "_", frame_id, ".png");
  cv::Mat mask;
  cv::inRange(img, range_min, range_max, mask);
  saveImageColorMap(img, mask, filename, flag);
}

void saveImageColorMap(const cv::Mat &img, const cv::Mat &mask, const std::string &filename, ColormapType flag)
{
  cv::Mat map_colored;
  switch(flag)
  {
    case ColormapType::DEPTH:
      map_colored = analysis::convertToColorMapFromCVC1(img, mask, cv::COLORMAP_JET);
      break;
    case ColormapType::ELEVATION:
      map_colored = analysis::convertToColorMapFromCVC1(img, mask, cv::COLORMAP_JET);
      break;
    case ColormapType::NORMALS:
      map_colored = analysis::convertToColorMapFromCVC3(img, mask);
      break;
  }
  cv::imwrite(filename, map_colored);
}

void saveCvGridMapLayer(const CvGridMap &map, int zone, char band, const std::string &layer_name, const std::string &filename)
{
  cv::Mat data = map[layer_name];
  saveImage(data, filename);
  saveCvGridMapMeta(map, zone, band, filename.substr(0, filename.size()-3) + "yaml");
}

void saveCvGridMapMeta(const CvGridMap &map, int zone, char band, const std::string &filename)
{
  cv::FileStorage metafile(filename, cv::FileStorage::WRITE);
  metafile << "zone" << zone;
  metafile << "band" << band;
  metafile << "roi" << map.roi();
  metafile << "resolution" << map.resolution();
  metafile.release();
}

} // namespace io
} // namespace realm