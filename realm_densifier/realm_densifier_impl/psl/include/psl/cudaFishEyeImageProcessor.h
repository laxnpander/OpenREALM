// This file is part of PlaneSweepLib (PSL)

// Copyright 2016 Christian Haene (ETH Zuerich)

// PSL is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// PSL is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with PSL.  If not, see <http://www.gnu.org/licenses/>.

#ifndef CUDAFISHEYEIMAGEPROCESSOR_H
#define CUDAFISHEYEIMAGEPROCESSOR_H

#include "deviceImage.h"
#include <psl/fishEyeCameraMatrix.h>

namespace PSL_CUDA
{
    namespace CudaFishEyeImageProcessorDeviceCode
    {
        void fishEyeImageProcessorInitTexturing();
        void fishEyeImageProcessorUndistort(DeviceImage& inputImg, DeviceImage& outputImg, double k1_input,
                                            double k2_input, double p1_input, double p2_input,
                                            double k11_input, double k13_input, double k22_input, double k23_input,
                                            double k11inv_output, double k13inv_output, double k22inv_output, double k23inv_output);
        void fishEyeImageProcessorUndistortRectify(DeviceImage& inputImg, DeviceImage& outputImg, double xi_input, double k1_input,
                                                   double k2_input, double p1_input, double p2_input,
                                                   double k11_input, double k13_input, double k22_input, double k23_input,
                                                   double k11inv_output, double k13inv_output, double k22inv_output, double k23inv_output);
    }

    class CudaFishEyeImageProcessor
    {
    public:
        CudaFishEyeImageProcessor();
        void setInputImg(DeviceImage& inputImg, PSL::FishEyeCameraMatrix<double>& camera);
        std::pair<DeviceImage, PSL::FishEyeCameraMatrix<double> > undistort(double iScale, double fScale, double k1, double k2, double p1, double p2);
        DeviceImage extractPinhole(double iScale, Eigen::Matrix3d& Kpinhole, double k1, double k2, double p1, double p2);
    private:
        DeviceImage inputImg;
        PSL::FishEyeCameraMatrix<double> camera;
    };
}

#endif //CUDAFISHEYEIMAGEPROCESSOR_H
