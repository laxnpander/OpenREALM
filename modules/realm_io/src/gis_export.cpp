/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/imgproc.hpp>

#include <realm_core/loguru.h>
#include <realm_core/timer.h>
#include <realm_io/gis_export.h>

using namespace realm;

void io::saveGeoTIFF(const CvGridMap &map,
                     const std::string &color_layer_name,
                     const uint8_t &zone,
                     const std::string &filename,
                     bool do_build_overview,
                     bool do_split_save,
                     GDALProfile gdal_profile)
{
  long t = Timer::getCurrentTimeMilliseconds();

  cv::Mat img = map[color_layer_name];
  if (img.channels() == 4)
  {
    cv::cvtColor(img, img, CV_BGRA2BGR);
  }

  GDALDatasetMeta* meta = io::computeGDALDatasetMeta(map, color_layer_name, zone);

  if (!do_split_save || img.channels() == 1)
  {
    io::saveGeoTIFF(img, *meta, filename, do_build_overview, gdal_profile);

    LOG_F(INFO, "GeoTIFF saved, t = [%4.2f s], location: %s", (Timer::getCurrentTimeMilliseconds()-t)/1000.0, filename.c_str());
  }
  else
  {
    cv::Mat img_bands[img.channels()];
    cv::split(img, img_bands);

    for (int i = 0; i < img.channels(); ++i)
    {
      std::string filename_split = filename;
      switch(i)
      {
        case 0:
          filename_split.insert(filename_split.size()-4, "_b");
          break;
        case 1:
          filename_split.insert(filename_split.size()-4, "_g");
          break;
        case 2:
          filename_split.insert(filename_split.size()-4, "_r");
          break;
        default:
          throw(std::invalid_argument("Error: Exporting GeoTIFF split is only supported up to 3 channels."));
      }

      io::saveGeoTIFF(img_bands[i], *meta, filename_split, do_build_overview, gdal_profile);

      LOG_F(INFO, "GeoTIFF saved, t = [%4.2f s], location: %s", (Timer::getCurrentTimeMilliseconds()-t)/1000.0, filename_split.c_str());
    }
  }

  delete meta;
}

void io::saveGeoTIFF(const cv::Mat &data,
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
  OGRSpatialReference oSRS;
  GDALAllRegister();

  cv::Mat img_bands[data.channels()];
  cv::split(data, img_bands);

  // First a memory drive is created and the dataset loaded. This is done to provide the functionality of adding
  // internal overviews before translating the data to tif format. This is particularly used for Cloud Optimized GeoTIFFS,
  // see also: https://geoexamples.com/other/2019/02/08/cog-tutorial.html
  driver = GetGDALDriverManager()->GetDriverByName("MEM");
  dataset_mem = driver->Create(filename.c_str(), data.cols, data.rows, data.channels(), meta.datatype, options);

  char *pszSRS_WKT = nullptr;
  double geoinfo[6] = {meta.geoinfo[0], meta.geoinfo[1], meta.geoinfo[2], meta.geoinfo[3], meta.geoinfo[4], meta.geoinfo[5]};

  dataset_mem->SetGeoTransform(geoinfo);
  oSRS.SetUTM(meta.zone, TRUE);
  oSRS.SetWellKnownGeogCS("WGS84");
  oSRS.exportToWkt(&pszSRS_WKT);
  dataset_mem->SetProjection(pszSRS_WKT);
  CPLFree(pszSRS_WKT);

  for (uint8_t i = 1; i <= data.channels(); i++)
  {
    GDALRasterBand *band = dataset_mem->GetRasterBand(i);
    CPLErr error_code = band->RasterIO(GF_Write, 0, 0, data.cols, data.rows, img_bands[i - 1].data, data.cols, data.rows, meta.datatype, 0, 0);

    if (error_code != CE_None)
      throw(std::runtime_error("Error saving GeoTIFF: Unhandled error code."));

    if (data.type() == CV_8UC1 || data.type() == CV_8UC2 || data.type() == CV_8UC1 || data.type() == CV_8UC4)
      band->SetNoDataValue(0);
    else if (data.type() == CV_16UC1)
      band->SetNoDataValue(0);
    else if (data.type() == CV_32F)
      band->SetNoDataValue(std::numeric_limits<float>::quiet_NaN());
    else if (data.type() == CV_64F)
      band->SetNoDataValue(std::numeric_limits<double>::quiet_NaN());

    if (data.channels() == 1)
      band->SetColorInterpretation(GCI_GrayIndex);
    else
    {
      switch (i)
      {
        case 1:
          band->SetColorInterpretation(GCI_BlueBand);
          break;
        case 2:
          band->SetColorInterpretation(GCI_GreenBand);
          break;
        case 3:
          band->SetColorInterpretation(GCI_RedBand);
          break;
        case 4:
          band->SetColorInterpretation(GCI_AlphaBand);
          break;
        default:
          break;
      }
    }
  }

  if (do_build_overviews)
  {
    int overview_list[10] = { 2, 4, 8, 16, 32, 64, 128, 256, 1024, 2048 };
    dataset_mem->BuildOverviews("NEAREST", 10, overview_list, 0, nullptr, GDALDummyProgress, nullptr);
  }

  // The previously created dataset in memory is now finally translated to .tif format. All prior information is copied
  // and additional options added.
  driver = GetGDALDriverManager()->GetDriverByName(format);
  options = getExportOptionsGeoTIFF(gdal_profile);
  dataset_tif = GDALCreateCopy(driver, filename.c_str(), dataset_mem, 0, options, NULL, NULL);

  GDALClose((GDALDatasetH) dataset_mem);
  GDALClose(dataset_tif);
}

io::GDALDatasetMeta* io::computeGDALDatasetMeta(const CvGridMap &map, const std::string &layer_name, uint8_t zone)
{
  auto meta = new GDALDatasetMeta();

  cv::Rect2d roi = map.roi();
  double GSD = map.resolution();

  cv::Mat img = map[layer_name];

  if (img.type() == CV_8UC1 || img.type() == CV_8UC3 || img.type() == CV_8UC4)
    meta->datatype = GDT_Byte;
  else if (img.type() == CV_16UC1)
    meta->datatype = GDT_Int16;
  else if (img.type() == CV_32F)
    meta->datatype = GDT_Float32;
  else if (img.type() == CV_64F)
    meta->datatype = GDT_Float64;
  else
    throw(std::invalid_argument("Error saving GTiff: Image format not recognized!"));

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

char** io::getExportOptionsGeoTIFF(GDALProfile gdal_profile)
{
  char** options;
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