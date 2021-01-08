

#include <realm_core/loguru.h>
#include <realm_ortho/rectification.h>

using namespace realm;

CvGridMap::Ptr ortho::rectify(const Frame::Ptr &frame)
{
  // Check if all relevant layers are in the observed map
  CvGridMap::Ptr surface_model = frame->getSurfaceModel();

  if (!surface_model->exists("elevation") || (*surface_model)["elevation"].type() != CV_32F)
    throw(std::invalid_argument("Error: Layer 'elevation' does not exist or type is wrong."));

  // Apply rectification using the backprojection from grid
  CvGridMap::Ptr rectification =
      backprojectFromGrid(
          frame->getImageUndistorted(),
          *frame->getCamera(),
          surface_model->get("elevation"),
          surface_model->roi(),
          surface_model->resolution(),
          frame->getSurfaceAssumption() == SurfaceAssumption::ELEVATION
          );

  return rectification;
}

CvGridMap::Ptr ortho::backprojectFromGrid(
    const cv::Mat &img,
    const camera::Pinhole &cam,
    cv::Mat &surface,
    const cv::Rect2d &roi,
    double GSD,
    bool is_elevated,
    bool verbose)
{
  // Implementation details:
  // Implementation is chosen as a compromise between readability and performance. Especially the raw array operations
  // could be implemented in opencv. However depending on the resolution of the surface grid and the image the loop
  // iterations can go up to several millions. To keep the computation time as low as possible for this performance sink,
  // the style is as follows

  // Prepare projection, use raw arrays for performance
  cv::Mat cv_P = cam.P();
  double P[3][4] = {cv_P.at<double>(0, 0), cv_P.at<double>(0, 1), cv_P.at<double>(0, 2), cv_P.at<double>(0, 3),
                    cv_P.at<double>(1, 0), cv_P.at<double>(1, 1), cv_P.at<double>(1, 2), cv_P.at<double>(1, 3),
                    cv_P.at<double>(2, 0), cv_P.at<double>(2, 1), cv_P.at<double>(2, 2), cv_P.at<double>(2, 3)};

  // Prepare elevation angle calculation
  cv::Mat t_pose = cam.t();
  double t[3] = {t_pose.at<double>(0), t_pose.at<double>(1), t_pose.at<double>(2)};

  uchar is_elevated_val    = (is_elevated ? (uchar)0 : (uchar)255);
  cv::Mat valid            = (surface == surface);                      // Check for NaN values in the elevation
  cv::Mat color_data       = cv::Mat::zeros(surface.size(), CV_8UC4);   // contains BGRA color data
  cv::Mat elevation_angle  = cv::Mat::zeros(surface.size(), CV_32FC1);  // contains the observed elevation angle
  cv::Mat elevated         = cv::Mat::zeros(surface.size(), CV_8UC1);   // flag to set wether the surface has elevation info or not
  cv::Mat num_observations = cv::Mat::zeros(surface.size(), CV_16UC1);  // number of observations, should be one if it's a valid surface point

  LOG_IF_F(INFO, verbose, "Processing rectification:");
  LOG_IF_F(INFO, verbose, "- ROI (%f, %f, %f, %f)", roi.x, roi.y, roi.width, roi.height);
  LOG_IF_F(INFO, verbose, "- Dimensions: %i x %i", surface.rows, surface.cols);

  // Iterate through surface and project every cell to the image
  for (uint32_t r = 0; r < surface.rows; ++r)
    for (uint32_t c = 0; c < surface.cols; ++c)
    {
      if (!valid.at<uchar>(r, c))
      {
        continue;
      }

      auto elevation_val = static_cast<double>(surface.at<float>(r, c));

      double pt[3]{roi.x+(double)c*GSD, roi.y+roi.height-(double)r*GSD, elevation_val};
      double z = P[2][0]*pt[0]+P[2][1]*pt[1]+P[2][2]*pt[2]+P[2][3]*1.0;
      double x = (P[0][0]*pt[0]+P[0][1]*pt[1]+P[0][2]*pt[2]+P[0][3]*1.0)/z;
      double y = (P[1][0]*pt[0]+P[1][1]*pt[1]+P[1][2]*pt[2]+P[1][3]*1.0)/z;

      if (x > 0.0 && x < img.cols && y > 0.0 && y < img.rows)
      {
        color_data.at<cv::Vec4b>(r, c)      = img.at<cv::Vec4b>((int)y, (int)x);
        elevation_angle.at<float>(r, c)     = static_cast<float>(ortho::internal::computeElevationAngle(t, pt));
        elevated.at<uchar>(r, c)            = is_elevated_val;
        num_observations.at<uint16_t>(r, c) = 1;
      }
      else
      {
        surface.at<float>(r, c)             = std::numeric_limits<float>::quiet_NaN();
      }
    }

  LOG_IF_F(INFO, verbose, "Image successfully rectified.");

  auto rectification = std::make_shared<CvGridMap>(roi, GSD);
  rectification->add("color_rgb", color_data);
  rectification->add("elevation_angle", elevation_angle);
  rectification->add("elevated", elevated);
  rectification->add("num_observations", num_observations);
  return rectification;
}

double ortho::internal::computeElevationAngle(double *t, double *p)
{
  double v[3]{t[0]-p[0], t[1]-p[1], t[2]-p[2]};
  double v_length = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  return acos(sqrt(v[0]*v[0]+v[1]*v[1])/v_length)*180/3.1415;
}