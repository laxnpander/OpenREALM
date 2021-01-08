

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <realm_core/stereo.h>

void realm::stereo::computeRectification(const Frame::Ptr &frame_left,
                                         const Frame::Ptr &frame_right,
                                         cv::Mat &R1,
                                         cv::Mat &P1,
                                         cv::Mat &R2,
                                         cv::Mat &P2,
                                         cv::Mat &Q)
{
  assert(frame_left->isImageResizeSet() && frame_right->isImageResizeSet());
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
  // Compute rectification parameters
  cv::Rect* roi_left = nullptr;
  cv::Rect* roi_right = nullptr;
  cv::stereoRectify(K_l, D_l, K_r, D_r, frame_left->getResizedImageSize(), R, t, R1, R2, P1, P2, Q,
                    cv::CALIB_ZERO_DISPARITY, -1, frame_left->getResizedImageSize(), roi_left, roi_right);
}

void realm::stereo::remap(const Frame::Ptr &frame,
                          const cv::Mat &R,
                          const cv::Mat &P,
                          cv::Mat &img_remapped)
{
  // inner calib
  cv::Mat K = frame->getResizedCalibration();
  // distortion
  cv::Mat D = frame->getCamera()->distCoeffs();
  // remapping
  cv::Mat map11, map12;
  initUndistortRectifyMap(K, D, R, P, frame->getResizedImageSize(), CV_16SC2, map11, map12);
  // Get image and convert to grayscale
  cv::Mat img = frame->getResizedImageUndistorted();
  cvtColor(img, img, CV_BGR2GRAY);
  // Compute remapping
  cv::remap(img, img_remapped, map11, map12, cv::INTER_LINEAR);
}

cv::Mat realm::stereo::reprojectDepthMap(const camera::Pinhole::ConstPtr &cam,
                                         const cv::Mat &depthmap)
{
  // Chosen formula for reprojection follows the linear projection model:
  // x = K*(R|t)*X
  // R^T*K^-1*x-R^T*t = X
  // If pose is defined as "camera to world", then this formula simplifies to
  // R*K^-1*x+t=X

  if (depthmap.rows != cam->height() || depthmap.cols != cam->width())
    throw(std::invalid_argument("Error: Reprojecting depth map failed. Dimension mismatch!"));
  if (depthmap.type() != CV_32F)
    throw(std::invalid_argument("Error: Reprojecting depth map failed. Matrix has wrong type. It is expected to have type CV_32F."));

  // Implementation is chosen to be raw array, because it saves round about 30% computation time
  double fx = cam->fx();
  double fy = cam->fy();
  double cx = cam->cx();
  double cy = cam->cy();
  cv::Mat R_c2w = cam->R();
  cv::Mat t_c2w = cam->t();

  if (fabs(fx) < 10e-6 || fabs(fy) < 10-6 || fabs(cx) < 10e-6 || fabs(cy) < 10e-6)
    throw(std::invalid_argument("Error: Reprojecting depth map failed. Camera model invalid!"));

  if (R_c2w.empty() || t_c2w.empty())
    throw(std::invalid_argument("Error: Reprojecting depth map failed. Pose matrix is empty!"));

  // Array preparation
  double ar_R_c2w[3][3];
  for (uint8_t r = 0; r < 3; ++r)
    for (uint8_t c = 0; c < 3; ++c)
      ar_R_c2w[r][c] = R_c2w.at<double>(r, c);

  double ar_t_c2w[3];
  for (uint8_t r = 0; r < 3; ++r)
      ar_t_c2w[r] = t_c2w.at<double>(r);

  // Iteration
  cv::Mat img3d(depthmap.rows, depthmap.cols, CV_64FC3);
  for (int r = 0; r < depthmap.rows; ++r)
    for (int c = 0; c < depthmap.cols; ++c)
    {
      cv::Vec3d pt(0, 0, 0);

      auto depth = static_cast<double>(depthmap.at<float>(r, c));

      if (depth > 0)
      {
        // K^-1*(dc,dr,d)
        double u = (c - cx)*depth/fx;
        double v = (r - cy)*depth/fy;

        pt[0] = ar_R_c2w[0][0]*u + ar_R_c2w[0][1]*v + ar_R_c2w[0][2]*depth + ar_t_c2w[0];
        pt[1] = ar_R_c2w[1][0]*u + ar_R_c2w[1][1]*v + ar_R_c2w[1][2]*depth + ar_t_c2w[1];
        pt[2] = ar_R_c2w[2][0]*u + ar_R_c2w[2][1]*v + ar_R_c2w[2][2]*depth + ar_t_c2w[2];
      }
      img3d.at<cv::Vec3d>(r, c) = pt;
    }
  return img3d;
}

cv::Mat realm::stereo::computeDepthMapFromPointCloud(const camera::Pinhole::ConstPtr &cam, const cv::Mat &points)
{
  /*
   * Depth computation according to [Hartley2004] "Multiple View Geometry in Computer Vision", S.162 for normalized
   * camera matrix
   */

  if (points.type() != CV_64F)
    throw(std::invalid_argument("Error: Computing depth map from point cloud failed. Point matrix type should be CV_64F!"));

  // Prepare depthmap dimensions
  uint32_t width = cam->width();
  uint32_t height = cam->height();

  // Prepare output data
  cv::Mat depth_map = cv::Mat(height, width, CV_32F, -1.0);

  // Prepare extrinsics
  cv::Mat T_w2c = cam->Tw2c();
  cv::Mat cv_R_w2c = T_w2c.row(2).colRange(0, 3).t();
  double R_w2c[3] = { cv_R_w2c.at<double>(0), cv_R_w2c.at<double>(1), cv_R_w2c.at<double>(2) };
  double zwc = T_w2c.at<double>(2, 3);

  // Prepare projection
  cv::Mat P_cv = cam->P();
  double P[3][4]{P_cv.at<double>(0, 0), P_cv.at<double>(0, 1), P_cv.at<double>(0, 2), P_cv.at<double>(0, 3),
                 P_cv.at<double>(1, 0), P_cv.at<double>(1, 1), P_cv.at<double>(1, 2), P_cv.at<double>(1, 3),
                 P_cv.at<double>(2, 0), P_cv.at<double>(2, 1), P_cv.at<double>(2, 2), P_cv.at<double>(2, 3)};

  for (int i = 0; i < points.rows; ++i)
  {
    auto pixel = points.ptr<double>(i);
    double pt_x = pixel[0];
    double pt_y = pixel[1];
    double pt_z = pixel[2];

    // Depth calculation
    double depth = R_w2c[0]*pt_x + R_w2c[1]*pt_y + R_w2c[2]*pt_z + zwc;

    // Projection to image with x = P * X
    double w =        P[2][0]*pt_x + P[2][1]*pt_y + P[2][2]*pt_z + P[2][3]*1.0;
    auto   u = (int)((P[0][0]*pt_x + P[0][1]*pt_y + P[0][2]*pt_z + P[0][3]*1.0)/w);
    auto   v = (int)((P[1][0]*pt_x + P[1][1]*pt_y + P[1][2]*pt_z + P[1][3]*1.0)/w);

    if (u >= 0 && u < width && v >= 0 && v < height)
    {
      if (depth > 0)
        depth_map.at<float>(v, u) = static_cast<float>(depth);
      else
        depth_map.at<float>(v, u) = -1.0f;
    }
  }
  return depth_map;
}

cv::Mat realm::stereo::computeNormalsFromDepthMap(const cv::Mat& depth)
{
  // We have to shrink the normal mat by two, because otherwise we would run into border issues as the Kernel as size 3x3
  cv::Mat normals(depth.rows-2, depth.cols-2, CV_32FC3);

  for(int r = 1; r < depth.rows-1; ++r)
    for(int c = 1; c < depth.cols-1; ++c)
    {
      float dzdx = (depth.at<float>(r, c+1) - depth.at<float>(r, c-1)) / 2.0f;
      float dzdy = (depth.at<float>(r+1, c) - depth.at<float>(r-1, c)) / 2.0f;

      cv::Vec3f d(-dzdx, -dzdy, 1.0f);
      normals.at<cv::Vec3f>(r-1, c-1) = d / cv::norm(d);
    }
  cv::copyMakeBorder(normals, normals, 1, 1, 1, 1, cv::BORDER_REFLECT);
  return normals;
}

double realm::stereo::computeBaselineFromPose(const cv::Mat &p1, const cv::Mat &p2)
{
  cv::Mat t1 = p1.col(3);
  cv::Mat t2 = p2.col(3);
  return cv::norm(t1-t2);
}