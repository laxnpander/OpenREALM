

#include <realm_io/realm_export.h>

namespace realm
{
namespace io
{

void saveTimestamp(uint64_t timestamp,
                   uint32_t frame_id,
                   const std::string &filename)
{
// Open file and write data
  std::ofstream file;
  file.open(filename.c_str(), std::ios::app);
  file << frame_id << " " << timestamp << std::endl;
}

void saveTrajectory(uint64_t timestamp,
                    const cv::Mat &pose,
                    const std::string &filepath)
{
  // Grab container data
  cv::Mat t = pose.col(3);
  cv::Mat R = pose.rowRange(0, 3).colRange(0, 3);

  Eigen::Matrix3d R_eigen;
  R_eigen << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
  Eigen::Quaterniond q(R_eigen);

  // Open file and write data
  std::ofstream file;
  file.open(filepath.c_str(), std::ios::app);
  saveTrajectoryTUM(&file, timestamp, t.at<double>(0), t.at<double>(1), t.at<double>(2), q.x(), q.y(), q.z(), q.w());
  file.close();
}

void saveTrajectoryTUM(std::ofstream *file,
                       uint64_t timestamp,
                       double x,
                       double y,
                       double z,
                       double qx,
                       double qy,
                       double qz,
                       double qw)
{
  // TUM file format
  (*file).precision(10);
  (*file) << timestamp << " " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
}

void saveCvGridMap(const CvGridMap &map, const std::string &filepath)
{
  std::string suffix = filepath.substr(filepath.size()-9, 9);

  if(suffix != ".grid.bin")
    throw(std::invalid_argument("Error saving CvGridMap to binary. Suffix not supported!"));

  FILE* file = fopen((filepath).c_str(), "wb");

  // Writing the ROI
  cv::Rect2d roi = map.roi();
  double roi_raw[4] = { roi.x, roi.y, roi.width, roi.height };
  fwrite(roi_raw, 4, sizeof(double), file);

  // Writing the resolution
  double resolution = map.resolution();
  fwrite(&resolution, 1, sizeof(double), file);

  std::vector<std::string> layer_names = map.getAllLayerNames();

  // Writing nrof layers
  int nrof_layers = layer_names.size();
  fwrite(&nrof_layers, 1, sizeof(int), file);

  for (const auto &layer_name : layer_names)
  {
    CvGridMap::Layer layer = map.getLayer(layer_name);

    // Writing layer name
    int length = layer_name.length();
    fwrite(&length, 1, sizeof(int), file);
    fwrite(layer_name.c_str(), length, sizeof(char), file);

    // Writing layer interpolation flag
    int interpolation = layer.interpolation;
    fwrite(&interpolation, 1, sizeof(int), file);

    // Writing layer data
    int elem_size_in_bytes = (int)layer.data.elemSize();
    int elem_type          = (int)layer.data.type();

    int size[4] = {layer.data.cols, layer.data.rows, elem_size_in_bytes, elem_type};
    fwrite(size, 4, sizeof(int), file);

    // Operating rowise, so even non-continuous matrices are properly written to binary
    for (int r = 0; r < layer.data.rows; ++r)
      fwrite(layer.data.ptr<void>(r), layer.data.cols, elem_size_in_bytes, file);
  }

  fclose(file);
}

} // namespace io
} // namespace realm