/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011 Jack O'Quin
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef PROJECT_UTM32_H
#define PROJECT_UTM32_H

#include <memory>
#include <limits>

namespace realm
{
/**
 * @brief UTM32-Point class
 * Adapted from: ROS geodesy package
 */

class UTMPose
{
    // Type definitions
  public:
    using Ptr = std::shared_ptr<UTMPose>;
    using ConstPtr = std::shared_ptr<const UTMPose>;

    // Class definition
  public:

    /** Null constructor. Makes a 2D, invalid point object. */
    UTMPose()
        : easting(0.0),
          northing(0.0),
          altitude(std::numeric_limits<double>::quiet_NaN()),
          heading(std::numeric_limits<double>::quiet_NaN()),
          zone(0),
          band(' ')
    {
    }

    /** Copy constructor. */
    UTMPose(const UTMPose &that) = default;

    /** Create a 3-D grid point. */
    UTMPose(double _easting,
             double _northing,
             double _altitude,
             double _heading,
             uint8_t _zone,
             char _band)
        : easting(_easting), northing(_northing), altitude(_altitude), heading(_heading), zone(_zone), band(_band)
    {
    }

    // data members
    double easting;           ///< easting within grid zone [meters]
    double northing;          ///< northing within grid zone [meters]
    double altitude;          ///< altitude above ellipsoid [meters] or NaN
    double heading;
    uint8_t zone;             ///< UTM longitude zone number
    char band;              ///< MGRS latitude band letter

}; // class UTMPoint

static char UTMBand(double Lat, double Lon)
{
    char LetterDesignator;

    if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
    else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
    else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
    else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
    else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
    else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
    else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
    else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
    else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
    else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
    else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
    else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
    else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
    else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
    else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
    else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
    else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
    else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
    else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
    else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
        // '_' is an error flag, the Latitude is outside the UTM limits
    else LetterDesignator = ' ';

    return LetterDesignator;
}

}

#endif //PROJECT_UTM32_H
