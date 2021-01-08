

#include <realm_io/realm_import.h>

#include <eigen3/Eigen/Eigen>

using namespace realm;

camera::Pinhole io::loadCameraFromYaml(const std::string &filepath, double* fps)
{
  // Identify camera model
  CameraSettings::Ptr settings = CameraSettingsFactory::load(filepath);

  // Load camera informations depending on model
  if ((*settings)["type"].toString() == "pinhole")
  {
    // Grab the fps, as it is note saved in the camera container
    if (fps != nullptr)
      *fps = (*settings)["fps"].toDouble();

    // Create pinhole model
    camera::Pinhole cam((*settings)["fx"].toDouble(), (*settings)["fy"].toDouble(),
                        (*settings)["cx"].toDouble(), (*settings)["cy"].toDouble(),
              (uint32_t)(*settings)["width"].toInt(), (uint32_t)(*settings)["height"].toInt());
    cam.setDistortionMap((*settings)["k1"].toDouble(), (*settings)["k2"].toDouble(),
                         (*settings)["p1"].toDouble(), (*settings)["p2"].toDouble(), (*settings)["k3"].toDouble());
    return cam;
  }
  else
  {
      throw std::runtime_error("Unable to load camera settings from yaml file at: " + filepath);
  }

}

cv::Mat io::loadGeoreferenceFromYaml(const std::string &filepath)
{
  cv::Mat georeference;

  cv::FileStorage fs(filepath, cv::FileStorage::READ);
  fs["transformation_w2g"] >> georeference;
  fs.release();

  return georeference;
}

std::unordered_map<uint64_t, cv::Mat> io::loadTrajectoryFromTxtTUM(const std::string &directory,
                                                                   const std::string &filename)
{
  return io::loadTrajectoryFromTxtTUM(directory + "/" + filename);
};

std::unordered_map<uint64_t, cv::Mat> io::loadTrajectoryFromTxtTUM(const std::string &filepath)
{
  // Prepare result
  std::unordered_map<uint64_t, cv::Mat> result;

  // Open file
  std::ifstream file(filepath);
  if (!file.is_open())
    throw(std::runtime_error("Error loading trajectory file from '" + filepath + "': Could not open file!"));

  // Iterating through every line
  std::string str;
  while (std::getline(file, str))
  {
    // Tokenize input line
    std::vector<std::string> tokens = io::split(str.c_str(), ' ');
    if (tokens.size() < 7)
      throw(std::runtime_error("Error loading trajectory file from '\" + (directory+filename) + \"': Not enough arguments in line!"));

    // Convert all tokens to values
    uint64_t timestamp = std::stoul(tokens[0]);
    double x = std::stod(tokens[1]);
    double y = std::stod(tokens[2]);
    double z = std::stod(tokens[3]);
    double qx = std::stod(tokens[4]);
    double qy = std::stod(tokens[5]);
    double qz = std::stod(tokens[6]);
    double qw = std::stod(tokens[7]);

    // Convert Quaternions to Rotation matrix
    Eigen::Quaterniond quat(qw, qx, qy, qz);
    Eigen::Matrix3d R_eigen = quat.toRotationMatrix();

    // Pose as 3x4 matrix
    cv::Mat pose = (cv::Mat_<double>(3, 4) << R_eigen(0, 0), R_eigen(0, 1), R_eigen(0, 2), x,
                                              R_eigen(1, 0), R_eigen(1, 1), R_eigen(1, 2), y,
                                              R_eigen(2, 0), R_eigen(2, 1), R_eigen(2, 2), z);
    result[timestamp] = pose;
  }
  return result;
};

cv::Mat io::loadSurfacePointsFromTxt(const std::string &filepath)
{
  // Prepare result
  cv::Mat points;

  // Open file
  std::ifstream file(filepath);
  if (!file.is_open())
    throw(std::runtime_error("Error loading surface point file from '" + filepath + "': Could not open file!"));

  // Iterating through every line
  std::string str;
  while (std::getline(file, str))
  {
    // Tokenize input line
    std::vector<std::string> tokens = io::split(str.c_str(), ' ');
    if (tokens.size() < 2)
      throw(std::runtime_error("Error loading surface point file from '" + filepath + "': Not enough arguments in line!"));

    // Convert all tokens to values
    double x = std::stod(tokens[0]);
    double y = std::stod(tokens[1]);
    double z = std::stod(tokens[2]);

    // Point as 1x3 mat
    cv::Mat pt = (cv::Mat_<double>(1, 3) << x, y, z);
    points.push_back(pt);
  }
  return points;
}

CvGridMap::Ptr io::loadCvGridMap(const std::string &filepath)
{
  if (!io::fileExists(filepath))
    throw(std::invalid_argument("Error loading image: File does not exist!"));

  std::string suffix = filepath.substr(filepath.size()-9, 9);

  if(suffix != ".grid.bin")
    throw(std::invalid_argument("Error loading CvGridMap: Unknown suffix"));

  FILE* file = fopen(filepath.c_str(), "rb");

  size_t elements_read;

  double x, y, width, height;
  double resolution;

  elements_read = fread(&x, sizeof(double), 1, file);
  elements_read = fread(&y, sizeof(double), 1, file);
  elements_read = fread(&width, sizeof(double), 1, file);
  elements_read = fread(&height, sizeof(double), 1, file);
  elements_read = fread(&resolution, sizeof(double), 1, file);

  int nrof_layers;

  elements_read = fread(&nrof_layers, sizeof(int), 1, file);

  auto map = std::make_shared<CvGridMap>(cv::Rect2d(x, y, width, height), resolution);

  for (int i = 0; i < nrof_layers; ++i)
  {
    int length;
    elements_read = fread(&length, sizeof(int), 1, file);

    char layer_name[length];
    elements_read = fread(&layer_name, sizeof(char), length, file);

    int interpolation;
    elements_read = fread(&interpolation, sizeof(int), 1, file);

    int header[4];
    elements_read = fread(header, sizeof(int), 4, file);

    if (elements_read != 4)
      throw(std::runtime_error("Error reading binary: Elements read do not match matrix dimension!"));

    int cols               = header[0];
    int rows               = header[1];
    int elem_size_in_bytes = header[2];
    int elem_type          = header[3];

    cv::Mat data = cv::Mat::ones(rows, cols, elem_type);

    elements_read = fread(data.data, elem_size_in_bytes, (size_t)(cols * rows), file);

    if (elements_read != (size_t)(cols * rows))
      throw(std::runtime_error("Error reading binary: Elements read do not match matrix dimension!"));

    map->add(std::string(layer_name), data, interpolation);
  }

  fclose(file);

  return map;
}