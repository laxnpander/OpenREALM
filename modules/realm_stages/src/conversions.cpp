

#include <realm_stages/conversions.h>

namespace realm
{

cv::Mat cvtToPointCloud(const cv::Mat &img3d, const cv::Mat &color, const cv::Mat &normals, const cv::Mat &mask)
{
  if (img3d.empty())
    throw(std::invalid_argument("Error: Depth map empty. Conversion to point cloud failed."));

  size_t n = (size_t)img3d.cols*img3d.rows;

  cv::Mat points = img3d.reshape(1, n);

  if (!color.empty())
  {
    cv::Mat color_reshaped = color.reshape(1, n);
    color_reshaped.convertTo(color_reshaped, CV_64FC1);

    if(color.channels() < 4)
    {
      cv::hconcat(points, color_reshaped, points);
    }
    else if (color.channels() == 4)
    {
      cv::hconcat(points, color_reshaped.colRange(0, 3), points);
    }
    else
    {
      throw(std::invalid_argument("Error converting depth map to point cloud: Color depth invalid."));
    }
  }

  if (!normals.empty())
  {
    cv::Mat normals_reshaped = normals.reshape(1, n);
    normals_reshaped.convertTo(normals_reshaped, CV_64FC1);

    cv::hconcat(points, normals_reshaped, points);
  }

  // Masking out undesired elements
  cv::Mat mask_reshaped = mask.reshape(1, n);

  cv::Mat points_masked;
  points_masked.reserve(n);

  for (int r = 0; r < mask_reshaped.rows; ++r)
  {
    if (mask.at<uchar>(r) == 255)
      points_masked.push_back(points.row(r));
  }

  return cv::Mat(points_masked);

}

cv::Mat cvtToPointCloud(const CvGridMap &map,
                        const std::string &layer_elevation,
                        const std::string &layer_color,
                        const std::string &layer_normals,
                        const std::string &layer_mask)
{
  assert(map.exists(layer_elevation));
  assert(!layer_color.empty() ? map.exists(layer_color) : true);
  assert(!layer_normals.empty() ? map.exists(layer_normals) : true);
  assert(!layer_mask.empty() ? map.exists(layer_mask) : true);

  cv::Size2i size = map.size();
  size_t n = (size_t)size.width*size.height;

  cv::Mat points;
  points.reserve(n);

  // OPTIONAL
  cv::Mat color;
  if (map.exists(layer_color))
    color = map[layer_color];

  cv::Mat elevation_normal;
  if (map.exists(layer_normals))
    elevation_normal = map[layer_normals];

  cv::Mat mask;
  if (map.exists(layer_mask))
    mask = map[layer_mask];

  // Create one point per grid element
  cv::Mat img3d(size, CV_64FC3);
  for (int r = 0; r < size.height; ++r)
    for (int c = 0; c < size.width; ++c)
      img3d.at<cv::Point3d>(r, c) = map.atPosition3d(r, c, layer_elevation);

  return cvtToPointCloud(img3d, color, elevation_normal, mask);
}

std::vector<Face> cvtToMesh(const CvGridMap &map,
                            const std::string &layer_elevation,
                            const std::string &layer_color,
                            const std::vector<cv::Point2i> &vertex_ids)
{
  assert(!layer_elevation.empty() && map.exists(layer_elevation));
  assert(!layer_color.empty() ? map.exists(layer_color) : true);

  // OPTIONAL
  cv::Mat color;
  if (map.exists(layer_color))
    color = map[layer_color];

  // Create output vector of faces
  size_t count = 0;
  std::vector<Face> faces(vertex_ids.size()/3);

  for (size_t i = 0; i < vertex_ids.size(); i+=3, ++count)
    for (size_t j = 0; j < 3; ++j)
    {
      faces[count].vertices[j] = map.atPosition3d(vertex_ids[i+j].y, vertex_ids[i+j].x, layer_elevation);
      if (!color.empty())
        faces[count].color[j] = color.at<cv::Vec4b>(vertex_ids[i+j].y, vertex_ids[i+j].x);
      else
        faces[count].color[j] = cv::Vec4b(0, 0, 0, 255);
    }
  return faces;
}

} // namespace realm