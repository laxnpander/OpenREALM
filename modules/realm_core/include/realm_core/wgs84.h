

#ifndef PROJECT_WGS84_H
#define PROJECT_WGS84_H

/*!
 * @brief Class for WGS84 coordinates. Currently we avoid GDAL dependencies in the realm_core library, therefore no
 *        conversion is provided here. That might change in the future.
 */

class WGSPose
{
  public:
    double latitude;
    double longitude;
    double altitude;
    double heading;
};

#endif //PROJECT_WGS84_H
