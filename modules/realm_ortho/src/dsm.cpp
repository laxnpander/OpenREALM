

#include <realm_ortho/dsm.h>

#include <opencv2/imgproc.hpp>

using namespace realm::ortho;

DigitalSurfaceModel::DigitalSurfaceModel(const cv::Rect2d &roi, double elevation)
: m_use_prior_normals(false),
  m_assumption(SurfaceAssumption::PLANAR),
  m_surface_normal_mode(SurfaceNormalMode::NONE)
{
  // Planar surface means an elevation of zero.
  // Therefore based region of interest a grid
  // map is created and filled with zeros.
  // Resolution is assumed to be 1.0m as default
  m_surface = std::make_shared<CvGridMap>();
  m_surface->setGeometry(roi, 1.0);
  m_surface->add("elevation", cv::Mat::ones(m_surface->size(), CV_32FC1) * elevation);
}

DigitalSurfaceModel::DigitalSurfaceModel(const cv::Rect2d &roi,
                                         const cv::Mat& points,
                                         SurfaceNormalMode mode,
                                         int knn_max_iter)
    : m_use_prior_normals(false),
      m_knn_max_iter(knn_max_iter),
      m_assumption(SurfaceAssumption::ELEVATION),
      m_surface_normal_mode(mode)
{
  // Elevation surface means, that prior information
  // about the surface is available. Here in the form
  // of a point cloud of observed points.
  // Strategy is now to:
  // 0) Filter input point cloud for outlier
  // 1) Create a Kd-tree of the observed point cloud
  // 2) Estimate resolution of the point cloud
  // 3) Create a grid map based on the previously computed resolution

  // Check if prior normals were computed and can be used
  if (points.cols >= 9)
    m_use_prior_normals = true;

  // 0) Filter input point cloud and create container
  cv::Mat points_filtered = filterPointCloud(points);

  m_point_cloud.pts.clear();
  m_point_cloud.pts.resize((unsigned long)points_filtered.rows);
  for (int i = 0; i < points_filtered.rows; ++i)
  {
    m_point_cloud.pts[i].x = points_filtered.at<double>(i, 0);
    m_point_cloud.pts[i].y = points_filtered.at<double>(i, 1);
    m_point_cloud.pts[i].z = points_filtered.at<double>(i, 2);
  }

  // 1) Init Kd-tree
  initKdTree(m_point_cloud);

  // 2) Estimate resolution based on the point cloud
  double GSD_estimated = computePointCloudGSD(m_point_cloud);

  // 3) Create grid map based on point cloud resolution and surface info
  m_surface = std::make_shared<CvGridMap>(roi, GSD_estimated);
  computeElevation(points_filtered);
}

cv::Mat DigitalSurfaceModel::filterPointCloud(const cv::Mat &points)
{
  assert(points.type() == CV_64F);
  assert(m_assumption == SurfaceAssumption::ELEVATION);

//  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//  for (uint32_t i = 0; i < points.rows; ++i)
//  {
//    pcl::PointXYZ pt;
//
//    // Position informations are saved in 0,1,2
//    pt.x = (float) points.at<double>(i, 0);
//    pt.y = (float) points.at<double>(i, 1);
//    pt.z = (float) points.at<double>(i, 2);
//
//    point_cloud->points.push_back(pt);
//  }
//  point_cloud->width = (int)point_cloud->points.size();
//  point_cloud->height = 1;
//
//  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_removal;
//  outlier_removal.setInputCloud(point_cloud);
//  outlier_removal.setMeanK(6);
//  outlier_removal.setStddevMulThresh(1.0);
//  outlier_removal.filter(*point_cloud_filtered);
//
//  cv::Mat points_filtered;
//  points_filtered.reserve(point_cloud_filtered->points.size());
//
//  for (const auto &pt_pcl : point_cloud_filtered->points)
//  {
//     cv::Mat pt_cv(1, 9, CV_64F);
//     pt_cv.at<double>(0) = pt_pcl.x;
//     pt_cv.at<double>(1) = pt_pcl.y;
//     pt_cv.at<double>(2) = pt_pcl.z;
//     points_filtered.push_back(pt_cv);
//     pt_cv.at<double>(3) = pt_pcl.r/255.0;
//     pt_cv.at<double>(4) = pt_pcl.g/255.0;
//     pt_cv.at<double>(5) = pt_pcl.b/255.0;
//     pt_cv.at<double>(6) = pt_pcl.normal_x;
//     pt_cv.at<double>(7) = pt_pcl.normal_y;
//     pt_cv.at<double>(8) = pt_pcl.normal_z;
//  }

  return points;
}

void DigitalSurfaceModel::initKdTree(const PointCloud<double> &point_cloud)
{
  assert(m_assumption == SurfaceAssumption::ELEVATION);

  // Build kd-tree for space hierarchy
  m_point_cloud_adaptor.reset(new PointCloudAdaptor_t(m_point_cloud));
  m_kd_tree.reset(new DigitalSurfaceModel::KdTree_t(kDimensionKdTree, *m_point_cloud_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf)));
  m_kd_tree->buildIndex();
}

double DigitalSurfaceModel::computePointCloudGSD(const PointCloud<double> &point_cloud)
{
  // Prepare container
  size_t n = point_cloud.pts.size();
  std::vector<double> dists;
  std::vector<size_t> tmp_indices(2);
  std::vector<double> tmp_dists(2);

  // Iterate through the point cloud and compute nearest neighbour distance
  auto n_iter = static_cast<size_t>(0.01 * n);
  dists.reserve(n/n_iter+1);
  for (size_t i = 0; i < n; i+=n_iter)
  {
    PointCloud<double>::Point pt = point_cloud.pts[i];

    // Preparation of output container
    tmp_indices.clear();
    tmp_dists.clear();

    // Initialization
    const double query_pt[3]{pt.x, pt.y, 0.0};

    m_kd_tree->knnSearch(&query_pt[0], 2u, &tmp_indices[0], &tmp_dists[0]);

    // "closest point" distance is zero, because search point is also trained data
    // Therefore choose the one point that distance is above approx 0.0
    if (tmp_dists[0] > 10e-5)
      dists.push_back(tmp_dists[0]);
    else if (tmp_dists[1] > 10e-5)
      dists.push_back(tmp_dists[1]);
  }
  // Compute and return average dist
  return sqrt(accumulate(dists.begin(), dists.end(), 0.0)/dists.size());
}

void DigitalSurfaceModel::computeElevation(const cv::Mat &point_cloud)
{
  assert(m_assumption == SurfaceAssumption::ELEVATION);

  cv::Size2i size = m_surface->size();

  // Essential layers / must have
  cv::Mat elevation = cv::Mat(size, CV_32FC1, std::numeric_limits<float>::quiet_NaN());

  // Optional computation according to flag
  cv::Mat elevation_normal(size, CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));

  for (uint32_t r = 0; r < size.height; ++r)
    for (uint32_t c = 0; c < size.width; ++c)
    {
      cv::Point2d pt = m_surface->atPosition2d(r, c);
      std::vector<double> query_pt{pt.x, pt.y, 0.0};

      // Prepare kd-search for nearest neighbors
      double resolution = m_surface->resolution();
      std::vector<std::pair<int, double>> indices_dists;

      // Process neighbor search, if no neighbors are found, extend search distance
      for (int i = 0; i < m_knn_max_iter; ++i)
      {
        nanoflann::RadiusResultSet<double, int> result_set(static_cast<double>(i) * resolution, indices_dists);
        m_kd_tree->findNeighbors(result_set, &query_pt[0], nanoflann::SearchParams());

        // Process only if neighbours were found
        if (result_set.size() >= 3u)
        {
          std::vector<double> distances;
          std::vector<double> heights;
          std::vector<PlaneFitter::Point> points;
          std::vector<PlaneFitter::Normal> normals_prior;
          for (const auto &s : result_set.m_indices_dists)
          {
            distances.push_back(s.second);
            heights.push_back(point_cloud.at<double>(s.first, 2));
            points.emplace_back(PlaneFitter::Point{point_cloud.at<double>(s.first, 0),
                                                   point_cloud.at<double>(s.first, 1),
                                                   point_cloud.at<double>(s.first, 2)});
            if (point_cloud.cols >= 9)
              normals_prior.emplace_back(PlaneFitter::Normal{point_cloud.at<double>(s.first, 6),
                                                             point_cloud.at<double>(s.first, 7),
                                                             point_cloud.at<double>(s.first, 8)});
          }

          elevation.at<float>(r, c) = interpolateHeight(heights, distances);

          if ((m_surface_normal_mode == SurfaceNormalMode::NONE) && m_use_prior_normals)
            elevation_normal.at<cv::Vec3f>(r, c) = interpolateNormal(normals_prior, distances);
          else if (m_surface_normal_mode != SurfaceNormalMode::NONE)
            elevation_normal.at<cv::Vec3f>(r, c) = computeSurfaceNormal(points, distances);

          break;
        }
      }
    }

  m_surface->add("elevation", elevation);

  // If exact normal computation was chosen, remove high frequent noise from data
  if (m_surface_normal_mode == SurfaceNormalMode::RANDOM_NEIGHBOURS
      || m_surface_normal_mode == SurfaceNormalMode::FURTHEST_NEIGHBOURS)
    cv::medianBlur(elevation_normal, elevation_normal, 5);

  // If normals were computed. set in surface map
  if (m_surface_normal_mode != SurfaceNormalMode::NONE || m_use_prior_normals)
    m_surface->add("elevation_normal", elevation_normal);
}

realm::CvGridMap::Ptr DigitalSurfaceModel::getSurfaceGrid()
{
  return m_surface;
}

float DigitalSurfaceModel::interpolateHeight(const std::vector<double> &heights, const std::vector<double> &dists)
{
  double numerator = 0.0;
  double denominator = 0.0;
  for (size_t i = 0u; i < heights.size(); ++i)
  {
    numerator += heights[i] / dists[i];
    denominator += 1.0 / dists[i];
  }
  return static_cast<float>(numerator / denominator);
}

cv::Vec3f DigitalSurfaceModel::interpolateNormal(const std::vector<PlaneFitter::Normal> &normals, const std::vector<double> &dists)
{
  cv::Vec3f numerator(0.0f, 0.0f, 0.0f);
  double denominator = 0.0;
  for (size_t i = 0u; i < dists.size(); ++i)
  {
    numerator[0] += normals[i].x / dists[i];
    numerator[1] += normals[i].y / dists[i];
    numerator[2] += normals[i].z / dists[i];
    denominator += 1.0 / dists[i];
  }
  return cv::normalize(numerator / static_cast<float>(denominator));
}

cv::Vec3f DigitalSurfaceModel::computeSurfaceNormal(const std::vector<PlaneFitter::Point> &points, const std::vector<double> &dists)
{
  assert(m_surface_normal_mode != SurfaceNormalMode::NONE);

  // Creation of plane fitter
  PlaneFitter plane_fitter;

  // Vector to save point indices that are used for normal computation
  std::vector<size_t> indices_points;
  std::vector<PlaneFitter::Point> points_selected;

  switch(m_surface_normal_mode)
  {
    case SurfaceNormalMode::NONE:
      // Should be unreachable code segment
      throw(std::runtime_error("Error computing surface normal: Function called despite mode set to 'NONE'"));

    case SurfaceNormalMode::RANDOM_NEIGHBOURS:
      // Select the first three points found
      indices_points = std::vector<size_t>{0, 1, 2};
      points_selected = std::vector<PlaneFitter::Point>{
          points[indices_points[0]],
          points[indices_points[1]],
          points[indices_points[2]]};
      break;

    case SurfaceNormalMode::FURTHEST_NEIGHBOURS:
      // Select furthest points found
      indices_points = getKmaxElementsIndices(dists, 3);
      points_selected = std::vector<PlaneFitter::Point>{
          points[indices_points[0]],
          points[indices_points[1]],
          points[indices_points[2]]};
      break;

    case SurfaceNormalMode::BEST_FIT:
      // All points used for best fit
      points_selected = points;
      break;
  }

  // In case of 3 points no best fit is computed -> exact solution
  PlaneFitter::Plane plane = plane_fitter.estimate(points_selected);

  // Ensure surface points up
  if (plane.n.z < 0)
    return cv::Vec3f((float)-plane.n.x, (float)-plane.n.y, (float)-plane.n.z);
  else
    return cv::Vec3f((float)plane.n.x, (float)plane.n.y, (float)plane.n.z);
}

std::vector<size_t> DigitalSurfaceModel::getKmaxElementsIndices(std::vector<double> vec, size_t k)
{
  size_t n = vec.size();
  if (k == n)
    return std::vector<size_t>{0, 1, 2};
  else if(k > n)
    throw(std::out_of_range("Error computing kMaxElementsIndices: Not enough input values."));

  std::vector<size_t> result; result.reserve(k);
  std::vector<size_t> indices_sorted = sort_indices(vec);

  for (size_t i = n; n - k < i; --i)
    result.push_back(indices_sorted[i-1]);

  return result;
}