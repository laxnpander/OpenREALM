

#ifndef PROJECT_CV_EXPORT_H
#define PROJECT_CV_EXPORT_H

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <realm_core/frame.h>
#include <realm_core/analysis.h>
#include <realm_io/utilities.h>

namespace realm
{
namespace io
{

enum class ColormapType
{
  DEPTH,
  ELEVATION,
  NORMALS,
  NUM_OBS
};

void saveStereoPair(const Frame::Ptr &frame_left,
                    const Frame::Ptr &frame_right,
                    const std::string &path);

void saveImage(const cv::Mat &img,
               const std::string &name);

void saveImageToBinary(const cv::Mat &data,
                       const std::string &filepath);

void saveDepthMap(const Depthmap::Ptr &img,
                  const std::string &filename,
                  uint32_t id);

void saveImageColorMap(const cv::Mat &img,
                       float range_min,
                       float range_max,
                       const std::string &directory,
                       const std::string &name,
                       uint32_t frame_id,
                       ColormapType flag);

void saveImageColorMap(const cv::Mat &img,
                       const cv::Mat &mask,
                       const std::string &directory,
                       const std::string &name,
                       ColormapType flag);

void saveImageColorMap(const cv::Mat &img,
                       const cv::Mat &mask,
                       const std::string &directory,
                       const std::string &name,
                       uint32_t frame_id,
                       ColormapType flag);

void saveImageColorMap(const cv::Mat &img,
                       const cv::Mat &mask,
                       const std::string &name,
                       ColormapType flag);

void saveCvGridMapLayer(const CvGridMap &map,
                        int zone,
                        char band,
                        const std::string &layer_name,
                        const std::string &filename);

void saveCvGridMapMeta(const CvGridMap &map,
                       int zone,
                       char band,
                       const std::string &filename);

} // namespace io
} // namespace realm

#endif //PROJECT_CV_EXPORT_H
