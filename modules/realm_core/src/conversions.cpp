

#include <realm_core/conversions.h>

using namespace realm;

UTMPose gis::convertToUTM(const WGSPose &wgs)
{
  // TODO: Check if utm conversions are valid everywhere (not limited to utm32)
  // Compute zone
  double lon_tmp = (wgs.longitude+180)-int((wgs.longitude+180)/360)*360-180;
  auto zone = static_cast<int>(1 + (wgs.longitude+180.0)/6.0);
  if(wgs.latitude >= 56.0 && wgs.latitude < 64.0 && lon_tmp >= 3.0 && lon_tmp < 12.0)
    zone = 32;
  if(wgs.latitude >= 72.0 && wgs.latitude < 84.0)
  {
    if(      lon_tmp >= 0.0  && lon_tmp <  9.0 ) zone = 31;
    else if( lon_tmp >= 9.0  && lon_tmp < 21.0 ) zone = 33;
    else if( lon_tmp >= 21.0 && lon_tmp < 33.0 ) zone = 35;
    else if( lon_tmp >= 33.0 && lon_tmp < 42.0 ) zone = 37;
  }

  // Compute band
  char band = UTMBand(wgs.latitude, wgs.longitude);
  int is_northern = (wgs.latitude < 0.0 ? 0 : 1);

  OGRSpatialReference ogr_wgs;
  gis::initAxisMappingStrategy(&ogr_wgs);

  ogr_wgs.SetWellKnownGeogCS("WGS84");

  OGRSpatialReference ogr_utm;
  gis::initAxisMappingStrategy(&ogr_utm);

  ogr_utm.SetWellKnownGeogCS("WGS84");
  ogr_utm.SetUTM(zone, is_northern);

  OGRCoordinateTransformation* coord_trans = OGRCreateCoordinateTransformation(&ogr_wgs, &ogr_utm);

  double x = wgs.longitude;
  double y = wgs.latitude;
  bool result = coord_trans->Transform(1, &x, &y);

  // Underlying library is C malloc, have to clean up the resulting pointer to avoid a leak
  delete coord_trans;

  if (!result) {
    throw(std::runtime_error("Error converting utm coordinates to wgs84: Transformation failed"));
  }

  return UTMPose(x, y, wgs.altitude, wgs.heading, (uint8_t)zone, band);
}

WGSPose gis::convertToWGS84(const UTMPose &utm)
{
  OGRSpatialReference ogr_utm;
  gis::initAxisMappingStrategy(&ogr_utm);

  ogr_utm.SetWellKnownGeogCS("WGS84");
  ogr_utm.SetUTM(utm.zone, TRUE);

  OGRSpatialReference ogr_wgs;
  gis::initAxisMappingStrategy(&ogr_wgs);
  ogr_wgs.SetWellKnownGeogCS("WGS84");

  OGRCoordinateTransformation* coord_trans = OGRCreateCoordinateTransformation(&ogr_utm, &ogr_wgs);

  double x = utm.easting;
  double y = utm.northing;
  bool result = coord_trans->Transform(1, &x, &y);

  // Underlying library is C malloc, have to clean up the resulting pointer to avoid a leak
  delete coord_trans;

  if (!result) {
    throw(std::runtime_error("Error converting utm coordinates to wgs84: Transformation failed"));
  }

  return WGSPose{y, x, utm.altitude, utm.heading};
}

void gis::initAxisMappingStrategy(OGRSpatialReference *oSRS)
{
#if GDAL_VERSION_MAJOR >= 3
  {
    oSRS->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
  }
#endif
}