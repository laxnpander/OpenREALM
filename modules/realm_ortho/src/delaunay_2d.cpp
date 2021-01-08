

#include <realm_ortho/delaunay_2d.h>

using namespace realm;

Delaunay2D::Delaunay2D()
{

}

std::vector<cv::Point2i> Delaunay2D::buildMesh(const CvGridMap &grid, const std::string &mask)
{
  //CGAL boilerplate
  typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
  //We define a vertex_base with info. The "info" (size_t) allow us to keep track of the original point index.
  typedef CGAL::Triangulation_vertex_base_with_info_2<cv::Point2i, Kernel> VertexBase;
  typedef CGAL::Constrained_triangulation_face_base_2<Kernel> FaceBase;
  typedef CGAL::Triangulation_data_structure_2<VertexBase, FaceBase> TriangulationDataStruct;
  typedef CGAL::Delaunay_triangulation_2<Kernel, TriangulationDataStruct> DelaunayTriangulation;
  typedef DelaunayTriangulation::Point CGALPoint;

  cv::Mat valid;
  if (mask.empty())
    valid = cv::Mat::ones(grid.size(), CV_8UC1)*255;
  else
    valid = grid[mask];

  std::vector< std::pair<CGALPoint, cv::Point2i>> pts;
  pts.reserve(static_cast<size_t>(valid.rows * valid.cols));

  // Create and add vertices
  for (uint32_t r = 0; r < valid.rows; ++r)
    for (uint32_t c = 0; c < valid.cols; ++c)
      if (valid.at<uchar>(r, c) > 0)
      {
        cv::Point2d pt = grid.atPosition2d(r, c);
        pts.push_back(std::make_pair(CGALPoint(pt.x, pt.y), cv::Point2i(c, r)));
      }

  // The DT is built
  DelaunayTriangulation dt(pts.begin(), pts.end());

  std::vector<cv::Point2i> vertex_ids;
  vertex_ids.reserve(dt.number_of_faces()*3);
  if (vertex_ids.capacity() > 0)
  {
    for (auto it = dt.faces_begin(); it != dt.faces_end(); ++it)
      for (int i = 0; i < 3; ++i)
        vertex_ids.push_back(it->vertex(i)->info());
  }

  return vertex_ids;
}