

#include <eigen3/Eigen/Eigen>
#include <opencv2/core.hpp>

#include <realm_core/plane_fitter.h>

namespace realm
{

PlaneFitter::Plane PlaneFitter::estimate(const std::vector<PlaneFitter::Point> &points)
{
  size_t n = points.size();
  assert(n >= 3);

  if (n == 3)
  {
    // Exact solution with three points possible
    PlaneFitter::Point centroid = computeExactPlaneCentroid(points);
    PlaneFitter::Normal vn = computeExactPlaneNormal(points);
    return PlaneFitter::Plane{centroid, vn};
  }
  else
  {
    // Best fit / SVD decomposition with more than three points (over determined)
    Eigen::Matrix<Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic> points_eigen(3, n);

    for (size_t i = 0; i < n; ++i)
      points_eigen.col(i) << points[i].x, points[i].y, points[i].z;

    Eigen::Vector3d centroid(points_eigen.row(0).mean(), points_eigen.row(1).mean(), points_eigen.row(2).mean());

    points_eigen.row(0).array() -= centroid(0);
    points_eigen.row(1).array() -= centroid(1);
    points_eigen.row(2).array() -= centroid(2);

    auto svd = points_eigen.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d vn = svd.matrixU().rightCols<1>();

    return PlaneFitter::Plane{centroid(0), centroid(1), centroid(2), vn(0), vn(1), vn(2)};
  }
}

PlaneFitter::Normal PlaneFitter::computeExactPlaneNormal(const std::vector<PlaneFitter::Point> &points)
{
  assert(points.size() == 3);

  PlaneFitter::Vector v1{
      (points[1].x - points[0].x),
      (points[1].y - points[0].y),
      (points[1].z - points[0].z)};

  PlaneFitter::Vector v2{
      (points[2].x - points[0].x),
      (points[2].y - points[0].y),
      (points[2].z - points[0].z)};

  PlaneFitter::Vector v3{
          v1.y*v2.z - v1.z*v2.y,
          v1.z*v2.x - v1.x*v2.z,
          v1.x*v2.y - v1.y*v2.x};

  double length = sqrt(v3.x*v3.x + v3.y*v3.y + v3.z*v3.z);

  return PlaneFitter::Normal{
     v3.x/length,
     v3.y/length,
     v3.z/length};
}

PlaneFitter::Point PlaneFitter::computeExactPlaneCentroid(const std::vector<PlaneFitter::Point> &points)
{
  assert(points.size() == 3);
  return PlaneFitter::Point{
      (points[0].x + points[1].x + points[2].x) / 3.0,
      (points[0].y + points[1].y + points[2].y) / 3.0,
      (points[0].z + points[1].z + points[2].z) / 3.0};
}

} // namespace realm