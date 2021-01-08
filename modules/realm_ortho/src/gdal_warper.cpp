

#include <realm_ortho/gdal_warper.h>

#include <opencv2/highgui.hpp>

using namespace realm;

gis::GdalWarper::GdalWarper()
 : m_epsg_target(0),
   m_nrof_threads(-1)
{
  GDALAllRegister();

  // Warping is done in RAM and no data needs to be saved to the disk for now
  m_driver = GetGDALDriverManager()->GetDriverByName("MEM");
}

void gis::GdalWarper::setTargetEPSG(int epsg_code)
{
  m_epsg_target = epsg_code;
}

void gis::GdalWarper::setNrofThreads(int nrof_threads)
{
  m_nrof_threads = nrof_threads;
}

CvGridMap::Ptr gis::GdalWarper::warpRaster(const CvGridMap &map, uint8_t zone)
{
  //=======================================//
  //
  //      Step 1: Check validity
  //
  //=======================================//

  if (m_epsg_target == 0)
    throw(std::runtime_error("Error warping map: Target EPSG was not set!"));

  std::vector<std::string> layer_names = map.getAllLayerNames();
  if (layer_names.size() > 1)
    throw(std::invalid_argument("Error warping map: There is more than one layer in the map. This is currently not supported."));

  //=======================================//
  //
  //      Step 2: Prepare datasets
  //
  //=======================================//

  // Extract data and metadata
  cv::Mat data = map[layer_names[0]];
  io::GDALDatasetMeta* meta = io::computeGDALDatasetMeta(map, zone);

  // Convert the source map into a GDAL dataset
  GDALDataset* dataset_mem_src;
  dataset_mem_src = io::generateMemoryDataset(data, *meta);

  // Get source coordinate system.
  const char *gdal_proj_src = GDALGetProjectionRef(dataset_mem_src);
  CPLAssert(gdal_proj_src != NULL && strlen(gdal_proj_src) > 0);

  // Set target coordinate system
  char *gdal_proj_dst = nullptr;
  OGRSpatialReference oSRS;
  gis::initAxisMappingStrategy(&oSRS);

  oSRS.importFromEPSG(m_epsg_target);
  oSRS.exportToWkt(&gdal_proj_dst);
  CPLAssert(gdal_proj_dst != NULL && strlen(gdal_proj_dst) > 0);

  // Create a transformer that maps from source pixel/line coordinates
  // to destination georeferenced coordinates (not destination
  // pixel line).  We do that by omitting the destination dataset
  // handle (setting it to NULL).
  void *projector = GDALCreateGenImgProjTransformer(dataset_mem_src, gdal_proj_src, NULL, gdal_proj_dst, FALSE, 0, 1);
  CPLAssert( projector != NULL );

  // Get approximate output georeferenced bounds and resolution for file.
  double geoinfo_target[6];
  int warped_cols = 0, warped_rows = 0;
  CPLErr eErr = GDALSuggestedWarpOutput(dataset_mem_src, GDALGenImgProjTransform, projector, geoinfo_target, &warped_cols , &warped_rows  );
  CPLAssert( eErr == CE_None );
  GDALDestroyGenImgProjTransformer(projector);

  // Create the output object.
  GDALDataset* dataset_mem_dst = m_driver->Create("", warped_cols , warped_rows , data.channels(), meta->datatype, nullptr );
  CPLAssert( hDstDS != NULL );

  // Write out the projection definition.
  GDALSetProjection(dataset_mem_dst, gdal_proj_dst);
  GDALSetGeoTransform(dataset_mem_dst, geoinfo_target);

  CPLFree(gdal_proj_dst);

  //=======================================//
  //
  //      Step 3: Prepare warping
  //
  //=======================================//

  double no_data_value;

  switch(data.type() & CV_MAT_DEPTH_MASK)
  {
    case CV_32F:
      no_data_value = std::numeric_limits<float>::quiet_NaN();
      break;
    case CV_64F:
      no_data_value = std::numeric_limits<double>::quiet_NaN();
      break;
    default:
      no_data_value = 0.0;
  }

  GDALResampleAlg resample_alg;
  switch(map.getLayer(layer_names[0]).interpolation)
  {
    case cv::INTER_NEAREST:
      resample_alg = GRA_NearestNeighbour;
      break;
    case cv::INTER_LINEAR:
      resample_alg = GRA_Bilinear;
      break;
    case cv::INTER_CUBIC:
      resample_alg = GRA_Cubic;
      break;
    default:
      resample_alg = GRA_Bilinear;
  }

  char** warper_system_options = nullptr;
  warper_system_options = CSLSetNameValue(warper_system_options, "INIT_DEST", "NO_DATA");

  if (m_nrof_threads <= 0)
    warper_system_options = CSLSetNameValue(warper_system_options, "NUM_THREADS", "ALL_CPUS");
  else
    warper_system_options = CSLSetNameValue(warper_system_options, "NUM_THREADS", std::to_string(m_nrof_threads).c_str());

  // Setup warp options.
  GDALWarpOptions *warper_options = GDALCreateWarpOptions();
  warper_options->eResampleAlg = resample_alg;
  warper_options->papszWarpOptions = warper_system_options;
  warper_options->padfSrcNoDataReal = new double(no_data_value);
  warper_options->padfDstNoDataReal = new double(no_data_value);
  warper_options->hSrcDS = dataset_mem_src;
  warper_options->hDstDS = dataset_mem_dst;
  warper_options->nBandCount = 0;
  warper_options->nSrcAlphaBand = (data.channels() == 4 ? data.channels() : 0);
  warper_options->nDstAlphaBand = (data.channels() == 4 ? data.channels() : 0);

  // Establish reprojection transformer.
  warper_options->pTransformerArg = GDALCreateGenImgProjTransformer(
                                       dataset_mem_src,
                                       GDALGetProjectionRef(dataset_mem_src),
                                       dataset_mem_dst,
                                       GDALGetProjectionRef(dataset_mem_dst),
                                       FALSE, 0.0, 1 );

  warper_options->pfnTransformer = GDALGenImgProjTransform;

  //=======================================//
  //
  //      Step 4: Warping
  //
  //=======================================//

  GDALWarpOperation warping;
  warping.Initialize(warper_options);
  warping.ChunkAndWarpImage(0, 0, GDALGetRasterXSize(dataset_mem_dst), GDALGetRasterYSize(dataset_mem_dst));

  int raster_cols = dataset_mem_dst->GetRasterXSize();
  int raster_rows = dataset_mem_dst->GetRasterYSize();
  int raster_channels = dataset_mem_dst->GetRasterCount();

  int single_channel_type;
  switch(data.type() & CV_MAT_DEPTH_MASK)
  {
    case CV_8U:
      single_channel_type = CV_8UC1;
      break;
    case CV_16U:
      single_channel_type = CV_16UC1;
      break;
    case CV_32F:
      single_channel_type = CV_32FC1;
      break;
    case CV_64F:
      single_channel_type = CV_64FC1;
      break;
  }

  std::vector<cv::Mat> warped_data_split;
  for(int i = 1; i <= raster_channels; ++i)
  {
    // Save the channel in var not in the vector of Mat
    cv::Mat bckVar(raster_rows, raster_cols, single_channel_type);

    GDALRasterBand *band = dataset_mem_dst->GetRasterBand(i);
    band->SetNoDataValue(no_data_value);

    eErr = band->RasterIO(GF_Read, 0, 0, raster_cols, raster_rows, bckVar.data, raster_cols, raster_rows, band->GetRasterDataType(), 0, 0);
    CPLAssert( eErr == CE_None );

    fixGdalNoData(bckVar);

    warped_data_split.push_back(bckVar);
  }

  cv::Mat warped_data;
  cv::merge(warped_data_split, warped_data);

  GDALDestroyGenImgProjTransformer(warper_options->pTransformerArg );
  GDALDestroyWarpOptions(warper_options );
  delete meta;

  //=======================================//
  //
  //      Step 5: Compute output
  //
  //=======================================//

  double warped_geoinfo[6];
  dataset_mem_dst->GetGeoTransform(warped_geoinfo);

  double warped_resolution = warped_geoinfo[1];

  cv::Rect2d warped_roi;
  warped_roi.x = warped_geoinfo[0];
  warped_roi.y = warped_geoinfo[3] - warped_data.rows * warped_resolution;
  warped_roi.width = warped_data.cols * warped_resolution - warped_resolution;
  warped_roi.height = warped_data.rows * warped_resolution - warped_resolution;

  auto output = std::make_shared<CvGridMap>(warped_roi, warped_resolution);
  output->add(layer_names[0], warped_data, map.getLayer(layer_names[0]).interpolation);

  GDALClose(dataset_mem_dst);
  GDALClose(dataset_mem_src);

  return output;
}

/*CvGridMap::Ptr gis::GdalWarper::warpImage(const CvGridMap &map, uint8_t zone)
{
  //=======================================//
  //
  //      Step 1: Check validity
  //
  //=======================================//

  if (_epsg_target == 0)
    throw(std::runtime_error("Error warping map: Target EPSG was not set!"));

  //=======================================//
  //
  //      Step 2: Prepare datasets
  //
  //=======================================//

  // Get source coordinate system
  OGRSpatialReference src_SRS;
  src_SRS.SetUTM(zone, TRUE);
  src_SRS.SetWellKnownGeogCS("WGS84");

  // Set target coordinate system
  OGRSpatialReference dst_SRS;
  dst_SRS.importFromEPSG(_epsg_target);

  // Create transformator
  OGRCoordinateTransformation *tf = OGRCreateCoordinateTransformation(&src_SRS, &dst_SRS);

  cv::Rect2d roi = map.roi();

  double x[4] = { roi.x, roi.x,              roi.x + roi.width, roi.x + roi.width };
  double y[4] = { roi.y, roi.y + roi.height, roi.y,             roi.y + roi.height };

  bool success = tf->Transform(4, x, y);

  if (success)
  {
    double w = std::min({x[0], x[1], x[2], x[3]});
    double e = std::max({x[0], x[1], x[2], x[3]});
    double s = std::min({y[0], y[1], y[2], y[3]});
    double n = std::max({y[0], y[1], y[2], y[3]});
    cv::Rect2d roi_warped(w, s, e-w, n-s);

    cv::Mat src_points = (cv::Mat_<float>(4, 2) <<
                                                -roi.width/2, -roi.height/2,
        -roi.width/2,  roi.height/2,
        roi.width/2, -roi.height/2,
        roi.width/2,  roi.height/2);

    double resolution = map.resolution() * roi_warped.width/roi.width;
    double tx = roi_warped.x + roi_warped.width/2;
    double ty = roi_warped.y + roi_warped.height/2;

    cv::Mat dst_points(4, 2, CV_32F);
    for (int i = 0; i < 4; ++i)
    {
      dst_points.at<float>(i, 0) = (float)(x[i] - tx);
      dst_points.at<float>(i, 1) = (float)(y[i] - ty);
    }

    cv::Mat H = cv::getPerspectiveTransform(src_points, dst_points);

    CvGridMap map_clone = map.clone();
    auto map_warped = std::make_shared<CvGridMap>(roi_warped, resolution);

    for (const auto &layer_name : map.getAllLayerNames())
    {
      map_clone.changeResolution(resolution);
      const CvGridMap::Layer &layer = map_clone.getLayer(layer_name);

      cv::Mat data_warped;
      cv::warpPerspective(layer.data, data_warped, H, map_warped->size());

      map_warped->add(layer.name, data_warped, layer.interpolation);
    }

    return map_warped;
  }
  return nullptr;
}*/

void gis::GdalWarper::warpPoints()
{
  /*OGRSpatialReference s_SRS;
    const char* s_WKT = dataset_mem_src->GetProjectionRef();
    s_SRS.importFromWkt(const_cast<char **>(&s_WKT));
    OGRCoordinateTransformation *coordinate_transformation;

    double x = blub[0], y = blub[3];
    coordinate_transformation = OGRCreateCoordinateTransformation(&oSRS, &s_SRS);
    coordinate_transformation->Transform(1, &x, &y);*/
}

void gis::GdalWarper::fixGdalNoData(cv::Mat &data)
{
  if (data.type() == CV_32F)
  {
    cv::Mat mask;
    cv::inRange(data, -std::numeric_limits<float>::epsilon(), std::numeric_limits<float>::epsilon(), mask);
    data.setTo(std::numeric_limits<float>::quiet_NaN(), mask);
  }
  else if (data.type() == CV_64F)
  {
    cv::Mat mask;
    cv::inRange(data, -std::numeric_limits<double>::epsilon(), std::numeric_limits<double>::epsilon(), mask);
    data.setTo(std::numeric_limits<double>::quiet_NaN(), mask);
  }
}