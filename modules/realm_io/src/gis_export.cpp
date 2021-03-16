

#include <opencv2/imgproc.hpp>

#include <realm_core/loguru.h>
#include <realm_core/timer.h>
#include <realm_io/gis_export.h>

using namespace realm;

void io::saveGeoTIFF(const CvGridMap &map,
                     const uint8_t &zone,
                     const std::string &filename,
                     bool do_build_overview,
                     bool do_split_save,
                     GDALProfile gdal_profile)
{
  long t = Timer::getCurrentTimeMilliseconds();

  std::vector<std::string> layer_names = map.getAllLayerNames();
  if (layer_names.size() > 1)
    throw(std::invalid_argument("Error: Exporting Gtiff from CvGridMap is currently supported for single layer objects only."));

  cv::Mat img = map[map.getAllLayerNames()[0]];

  cv::Mat img_converted;
  if (img_converted.channels() == 3)
    cv::cvtColor(img, img_converted, cv::ColorConversionCodes::COLOR_BGR2RGB);
  else if (img.channels() == 4)
    cv::cvtColor(img, img_converted, cv::ColorConversionCodes::COLOR_BGRA2RGBA);
  else
    img_converted = img;

  GDALDatasetMeta* meta = io::computeGDALDatasetMeta(map, zone);

  if (!do_split_save || img_converted.channels() == 1)
  {
    io::saveGeoTIFFtoFile(img_converted, *meta, filename, do_build_overview, gdal_profile);

    LOG_F(INFO, "GeoTIFF saved, t = [%4.2f s], location: %s", (Timer::getCurrentTimeMilliseconds()-t)/1000.0, filename.c_str());
  }
  else
  {
    cv::Mat img_bands[img_converted.channels()];
    cv::split(img_converted, img_bands);

    for (int i = 0; i < img_converted.channels(); ++i)
    {
      std::string filename_split = filename;
      switch(i)
      {
        case 0:
          filename_split.insert(filename_split.size()-4, "_r");
          break;
        case 1:
          filename_split.insert(filename_split.size()-4, "_g");
          break;
        case 2:
          filename_split.insert(filename_split.size()-4, "_b");
          break;
        case 3:
          filename_split.insert(filename_split.size()-4, "_a");
          break;
        default:
          throw(std::invalid_argument("Error: Exporting GeoTIFF split is only supported up to 4 channels."));
      }

      io::saveGeoTIFFtoFile(img_converted, *meta, filename_split, do_build_overview, gdal_profile);

      LOG_F(INFO, "GeoTIFF saved, t = [%4.2f s], location: %s", (Timer::getCurrentTimeMilliseconds()-t)/1000.0, filename_split.c_str());

      t = Timer::getCurrentTimeMilliseconds();
    }
  }

  delete meta;
}

void io::saveGeoTIFFtoFile(const cv::Mat &data,
                           const GDALDatasetMeta &meta,
                           const std::string &filename,
                           bool do_build_overviews,
                           GDALProfile gdal_profile)
{
  const char *format = "GTiff";
  char **options = nullptr;

  GDALDriver* driver;
  GDALDataset* dataset_mem;
  GDALDatasetH dataset_tif;

  GDALAllRegister(); // -> This should be called only once according to the docs, move it in the future

  // First a memory drive is created and the dataset loaded. This is done to provide the functionality of adding
  // internal overviews before translating the data to tif format. This is particularly used for Cloud Optimized GeoTIFFS,
  // see also: https://geoexamples.com/other/2019/02/08/cog-tutorial.html
  dataset_mem = generateMemoryDataset(data, meta);

  if (do_build_overviews)
  {
    int overview_list[10] = { 2, 4, 8, 16, 32, 64, 128, 256, 1024, 2048 };
    dataset_mem->BuildOverviews("NEAREST", 10, overview_list, 0, nullptr, GDALDummyProgress, nullptr);
  }

  // The previously created dataset in memory is now finally translated to .tif format. All prior information is copied
  // and additional options added.
  driver = GetGDALDriverManager()->GetDriverByName(format);
  options = getExportOptionsGeoTIFF(gdal_profile);

  // Check if multi channel image, therefore RGB/BGR
  if (data.channels() == 3 || data.channels() == 4)
    options = CSLSetNameValue( options, "PHOTOMETRIC", "RGB" );

  dataset_tif = GDALCreateCopy(driver, filename.c_str(), dataset_mem, 0, options, NULL, NULL);

  GDALClose((GDALDatasetH) dataset_mem);
  GDALClose(dataset_tif);
}

io::GDALDatasetMeta* io::computeGDALDatasetMeta(const CvGridMap &map, uint8_t zone)
{
  auto meta = new GDALDatasetMeta();

  cv::Rect2d roi = map.roi();
  double GSD = map.resolution();

  cv::Mat img = map[map.getAllLayerNames()[0]];

  switch(img.type() & CV_MAT_DEPTH_MASK)
  {
    case CV_8U:
      meta->datatype = GDT_Byte;
      break;
    case CV_16U:
      meta->datatype = GDT_Int16;
      break;
    case CV_32F:
      meta->datatype = GDT_Float32;
      break;
    case CV_64F:
      meta->datatype = GDT_Float64;
      break;
    default:
      throw(std::invalid_argument("Error saving GTiff: Image format not recognized!"));
  }

  // Creating geo informations for GDAL and OGR
  meta->zone = zone;
  meta->geoinfo[0] = roi.x;
  meta->geoinfo[1] = GSD;
  meta->geoinfo[2] = 0.0;
  meta->geoinfo[3] = roi.y + roi.height;
  meta->geoinfo[4] = 0.0;
  meta->geoinfo[5] = -GSD;

  return meta;
}

GDALDataset* io::generateMemoryDataset(const cv::Mat &data, const io::GDALDatasetMeta &meta)
{
  GDALDriver* driver = nullptr;
  GDALDataset* dataset = nullptr;
  OGRSpatialReference oSRS;
  gis::initAxisMappingStrategy(&oSRS);
  
  char **options = nullptr;

  driver = GetGDALDriverManager()->GetDriverByName("MEM");
  dataset = driver->Create("", data.cols, data.rows, data.channels(), meta.datatype, options);

  char *pszSRS_WKT = nullptr;
  double geoinfo[6] = {meta.geoinfo[0], meta.geoinfo[1], meta.geoinfo[2], meta.geoinfo[3], meta.geoinfo[4], meta.geoinfo[5]};

  dataset->SetGeoTransform(geoinfo);
  oSRS.SetUTM(meta.zone, TRUE);
  oSRS.SetWellKnownGeogCS("WGS84");
  oSRS.exportToWkt(&pszSRS_WKT);
  dataset->SetProjection(pszSRS_WKT);
  CPLFree(pszSRS_WKT);

  cv::Mat img_bands[data.channels()];
  cv::split(data, img_bands);

  for (int i = 1; i <= data.channels(); i++)
  {
    GDALRasterBand *band = dataset->GetRasterBand(i);
    CPLErr error_code = band->RasterIO(GF_Write, 0, 0, data.cols, data.rows, img_bands[i - 1].data, data.cols, data.rows, meta.datatype, 0, 0);

    if (error_code != CE_None)
      throw(std::runtime_error("Error saving GeoTIFF: Unhandled error code."));

    setGDALBandNan(band, data);
  }
  return dataset;
}

char** io::getExportOptionsGeoTIFF(GDALProfile gdal_profile)
{
  char** options = nullptr;
  switch(gdal_profile)
  {
    case GDALProfile::COG:
      options = CSLSetNameValue( options, "INTERLEAVE", "PIXEL" );
      options = CSLSetNameValue( options, "TILED", "YES" );
      options = CSLSetNameValue( options, "BLOCKXSIZE", "256" );
      options = CSLSetNameValue( options, "BLOCKYSIZE", "256" );
      options = CSLSetNameValue( options, "PHOTOMETRIC", "MINISBLACK");
      options = CSLSetNameValue( options, "BIGTIFF", "IF_SAFER");
      options = CSLSetNameValue( options, "COPY_SRC_OVERVIEWS", "YES" );
      options = CSLSetNameValue( options, "COMPRESS", "LZW" );
      break;
    default:
      throw(std::invalid_argument("Error: Unknown GDAL export profile."));
  }
  return options;
}

void io::setGDALBandNan(GDALRasterBand *band, const cv::Mat &data)
{
  if (data.type() == CV_8UC1 || data.type() == CV_8UC2 || data.type() == CV_8UC3 || data.type() == CV_8UC4)
    band->SetNoDataValue(0);
  else if (data.type() == CV_16UC1)
    band->SetNoDataValue(0);
  else if (data.type() == CV_32F)
    band->SetNoDataValue(std::numeric_limits<float>::quiet_NaN());
  else if (data.type() == CV_64F)
    band->SetNoDataValue(std::numeric_limits<double>::quiet_NaN());
}