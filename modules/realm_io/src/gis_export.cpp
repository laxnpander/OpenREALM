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

#include <realm_io/gis_export.h>

using namespace realm;

void io::saveGeoTIFF(const CvGridMap &map,
                     const std::string &color_layer_name,
                     const uint8_t &zone,
                     const std::string &directory,
                     const std::string &name,
                     GDALProfile gdal_profile)
{
  // Create unique filename
  std::string filename = (directory + "/" + name + ".tif");
  io::saveGeoTIFF(map, color_layer_name, zone, filename, gdal_profile);
}

void io::saveGeoTIFF(const CvGridMap &map,
                     const std::string &color_layer_name,
                     const uint8_t &zone,
                     const std::string &directory,
                     const std::string &name,
                     uint32_t id,
                     GDALProfile gdal_profile)
{
  // Create unique filename
  std::string filename = io::createFilename(directory + "/" + name + "_", id, ".tif");
  io::saveGeoTIFF(map, color_layer_name, zone, filename, gdal_profile);
}

void io::saveGeoTIFF(const CvGridMap &map,
                 const std::string &color_layer_name,
                 const uint8_t &zone,
                 const std::string &filename,
                 GDALProfile gdal_profile)
{
  // Get relevant data from container class
  double GSD = map.resolution();
  cv::Rect2d roi = map.roi();
  cv::Mat color_layer = map[color_layer_name];

  // Creating geo informations for GDAL
  double geoproj[6];
  geoproj[0] = roi.x;
  geoproj[1] = GSD;
  geoproj[2] = 0;
  geoproj[3] = roi.y+roi.height;
  geoproj[4] = 0;
  geoproj[5] = -GSD;

  // Call to minimal saving function
  io::saveGeoTIFF(color_layer, filename.c_str(), geoproj, zone, gdal_profile);
}

void io::saveGeoTIFF(const cv::Mat &img,
                     const char *filename,
                     double *geoinfo,
                     const uint8_t &zone,
                     GDALProfile gdal_profile)
{
  const char *format = "GTiff";
  char **options = getExportOptionsGeoTIFF(gdal_profile);

  auto bands = static_cast<uint8_t>(img.channels());
  auto rows = static_cast<uint32_t>(img.rows);
  auto cols = static_cast<uint32_t>(img.cols);

  GDALDataset *dataset;
  GDALDriver *driver;
  OGRSpatialReference oSRS;

  GDALAllRegister();

  driver = GetGDALDriverManager()->GetDriverByName(format);

  GDALDataType pix_type;
  if (img.type() == CV_8UC1 || img.type() == CV_8UC3 || img.type() == CV_8UC4)
    pix_type = GDT_Byte;
  else if (img.type() == CV_16UC1)
    pix_type = GDT_Int16;
  else if (img.type() == CV_32F)
    pix_type = GDT_Float32;
  else if (img.type() == CV_64F)
    pix_type = GDT_Float64;
  else
    throw(std::invalid_argument("Error saving GTiff: Image format not recognized!"));

  dataset = driver->Create(filename, cols, rows, bands, pix_type, options);

  char *pszSRS_WKT = nullptr;

  dataset->SetGeoTransform(geoinfo);
  oSRS.SetUTM(zone, TRUE);
  oSRS.SetWellKnownGeogCS("WGS84");
  oSRS.exportToWkt(&pszSRS_WKT);
  dataset->SetProjection(pszSRS_WKT);
  CPLFree(pszSRS_WKT);

  cv::Mat img_bands[bands];
  cv::split(img, img_bands);

  for (uint8_t i = 1; i <= bands; i++)
  {
    GDALRasterBand *band = dataset->GetRasterBand(i);
    CPLErr error_code = band->RasterIO(GF_Write, 0, 0, cols, rows, img_bands[i - 1].data, cols, rows, pix_type, 0, 0);

    if (error_code != CE_None)
      throw(std::runtime_error("Error saving GeoTIFF: Unhandled error code."));

    if (img.type() == CV_8UC1 || img.type() == CV_8UC2 || img.type() == CV_8UC1 || img.type() == CV_8UC4)
      band->SetNoDataValue(0);
    else if (img.type() == CV_16UC1)
      band->SetNoDataValue(0);
    else if (img.type() == CV_32F)
      band->SetNoDataValue(std::numeric_limits<float>::quiet_NaN());
    else if (img.type() == CV_64F)
      band->SetNoDataValue(std::numeric_limits<double>::quiet_NaN());

    if (bands == 1)
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

  GDALClose((GDALDatasetH) dataset);
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
      options = CSLSetNameValue( options, "ZLEVEL", "1");
      options = CSLSetNameValue( options, "ZSTD_LEVEL", "9");
      options = CSLSetNameValue( options, "BIGTIFF", "IF_SAFER");
      options = CSLSetNameValue( options, "COPY_SRC_OVERVIEWS", "YES" );
      options = CSLSetNameValue( options, "COMPRESS", "LZW" );
      break;
    default:
      throw(std::invalid_argument("Error: Unknown GDAL export profile."));
  }
  return options;
}