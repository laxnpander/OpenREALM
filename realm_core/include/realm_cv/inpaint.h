/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2008-2012, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// COPY OF OPENCV 3.4 INPAINT
// Note: OpenCV 3.4 inpainting can handle floating point matrices, this is interesting for elevation and depth maps.
//       It is copied here to allow ROS kinetic user to use their standard OpenCV 3.3.1 version

#ifndef PROJECT_REALM_INPAINT_H
#define PROJECT_REALM_INPAINT_H

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/types_c.h>

namespace realm
{

enum
{
    INPAINT_NS
};

void inpaint(cv::InputArray _src, cv::InputArray _mask, cv::OutputArray _dst, double inpaintRange, int flags);
void cvInpaint(const CvArr* _input_img, const CvArr* _inpaint_mask, CvArr* _output_img, double inpaintRange, int flags);

} // namespace realm

#endif //PROJECT_INPAINT_H
