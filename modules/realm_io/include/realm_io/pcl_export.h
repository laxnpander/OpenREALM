

#ifndef PROJECT_PC_EXPORT_H
#define PROJECT_PC_EXPORT_H

#include <iostream>
#include <unordered_map>

#include <opencv2/core.hpp>

namespace realm
{

class CvGridMap;

namespace io
{

void saveElevationPointsToPLY(const CvGridMap &map,
                              const std::string &ele_layer_name,
                              const std::string &normals_layer_name,
                              const std::string &color_layer_name,
                              const std::string &mask_layer_name,
                              const std::string &directory,
                              const std::string &name);

void saveElevationPoints(const CvGridMap &map,
                         const std::string &ele_layer_name,
                         const std::string &normals_layer_name,
                         const std::string &color_layer_name,
                         const std::string &mask_layer_name,
                         const std::string &filename,
                         const std::string &suffix);

void saveElevationPointsRGB(const CvGridMap &map,
                            const std::string &ele_layer_name,
                            const std::string &color_layer_name,
                            const std::string &mask_layer_name,
                            const std::string &filename,
                            const std::string &suffix);

void saveElevationPointsRGBNormal(const CvGridMap &map,
                                  const std::string &ele_layer_name,
                                  const std::string &normals_layer_name,
                                  const std::string &color_layer_name,
                                  const std::string &mask_layer_name,
                                  const std::string &filename,
                                  const std::string &suffix);

void saveElevationMeshToPLY(const CvGridMap &map,
                            const std::vector<cv::Point2i> &vertices,
                            const std::string &ele_layer_name,
                            const std::string &normal_layer_name,
                            const std::string &color_layer_name,
                            const std::string &mask_layer_name,
                            const std::string &directory,
                            const std::string &name);

void saveElevationMeshToPLY(const CvGridMap &map,
                            const std::vector<cv::Point2i> &vertices,
                            const std::string &ele_layer_name,
                            const std::string &normal_layer_name,
                            const std::string &color_layer_name,
                            const std::string &mask_layer_name,
                            const std::string &filename);

} // namespace io
} // namespace realm



#endif //PROJECT_PC_EXPORT_H
