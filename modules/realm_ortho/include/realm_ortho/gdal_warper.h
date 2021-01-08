

#ifndef GENERAL_TESTBED_GDAL_WARPER_H
#define GENERAL_TESTBED_GDAL_WARPER_H

#include <realm_core/conversions.h>
#include <realm_io/gis_export.h>

#include <gdal/gdal_priv.h>
#include <gdal/gdalwarper.h>
#include <gdal/ogr_spatialref.h>
#include <gdal/cpl_conv.h>

namespace realm
{
namespace gis
{

/*!
 * @brief GdalWarper allows to transform an input CvGridMap from UTM coordinates into an EPSG standardized frame
 * utilizing GDAL library as backbone. The grid map is expected to fit into memory and must not contain more than
 * one layer. The number of threads used to compute the transformation can be provided.
 */
class GdalWarper
{
public:
  /*!
   * @brief Non-default constructor
   */
  GdalWarper();

  ~GdalWarper() = default;
  GdalWarper(const GdalWarper &other) = default;

  /*!
   * @brief Set the target coordinate frame in EPSG standard, see: https://en.wikipedia.org/wiki/EPSG_Geodetic_Parameter_Dataset
   * @param epsg_code EPSG Code, e.g. 3857 for EPSG:3857
   */
  void setTargetEPSG(int epsg_code);

  /*!
   * @brief Set the number of threads used to compute the projection to the target coordinate frame. Default is using
   * all CPU's.
   * @param nrof_threads Number of threads used to compute the projection
   */
  void setNrofThreads(int nrof_threads);

  /*!
   * @brief: Following the example in https://gdal.org/tutorials/warp_tut.html
   * The provided map must be in UTM coordinates and will be warped into the previously defined, target EPSG frame.
   * Only single layer maps are currently supported. The interpolation strategy is taken from the CvGridMap layer
   * meta information. No data values are kept as is, though there is currently an issue of GDAL setting no data values
   * for floating point data to 0.0. The current fix is to identify cells with value 0.0 and fill them to NaN after
   * warping. The resulting map is a deep copy.
   * @param map Map to be warped into a new coordinate system. Must not contain more than one layer.
   * @param zone UTM zone of the grid. TODO: should probably be solved smarter in the future
   * @return Warped map in the desired coordinate system. Coordinate system must be cartesian.
   */
  CvGridMap::Ptr warpRaster(const CvGridMap &map, uint8_t zone);

  //CvGridMap::Ptr warpImage(const CvGridMap &map, uint8_t zone);

  // TBD
  void warpPoints();

private:

  /// EPSG Code, e.g. 3857 for EPSG:3857
  int m_epsg_target;

  /// Number of threads used for projection
  int m_nrof_threads;

  /// Internal GDAL memory driver to create a dataset from the layer input
  GDALDriver* m_driver;

  /*!
   * @brief For some reason GDAL sets the no data value to 0.0 for floating point matrices, even though all options
   * are set to NaN. So for now this is a quickfix finding 0.0 values and setting them to NaN. Not ideal, because 0.0
   * can also be a valid point.
   * @param data Floating point matrix to be fixed
   */
  void fixGdalNoData(cv::Mat &data);
};

} // namespace gis
} // namespace realm

#endif //GENERAL_TESTBED_GDAL_WARPER_H
