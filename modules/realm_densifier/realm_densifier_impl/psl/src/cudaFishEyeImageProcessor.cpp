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

#include <utility>
#include <cmath>

#include <psl/cudaFishEyeImageProcessor.h>
#include <psl/common.h>

using namespace PSL_CUDA;
using namespace PSL;

CudaFishEyeImageProcessor::CudaFishEyeImageProcessor()
{
    CudaFishEyeImageProcessorDeviceCode::fishEyeImageProcessorInitTexturing();
}

void CudaFishEyeImageProcessor::setInputImg(DeviceImage& inputImg, FishEyeCameraMatrix<double>& camera)
{
    this->inputImg = inputImg;
    this->camera = camera;
    if (camera.getK()(0,1) != 0)
    {
        PSL_THROW_EXCEPTION("Only Ks without skew allowed.")
    }
}

std::pair<DeviceImage, PSL::FishEyeCameraMatrix<double> > CudaFishEyeImageProcessor::undistort(double iScale, double fScale, double k1, double k2, double p1, double p2)
{
    if (inputImg.getNumChannels() != 1)
    {
        PSL_THROW_EXCEPTION("Only grayscale supported.")
    }

    DeviceImage outputImage;
    int width = (int) round(iScale*inputImg.getWidth());
    int height = (int) round(iScale*inputImg.getHeight());

    outputImage.allocatePitched(width, height, 1);

    Eigen::Matrix3d Knew = camera.getK();
    Knew /= Knew(2,2); // make sure it is normalized
    Knew(0,0) *= iScale*fScale;
    Knew(1,1) *= iScale*fScale;
    Knew(0,2) = (Knew(0,2) + 0.5)*iScale - 0.5;
    Knew(1,2) = (Knew(1,2) + 0.5)*iScale -0.5;


    if (Knew(0,1) != 0)
    {
        PSL_THROW_EXCEPTION("Only Ks without skew allowed.")
    }

    Eigen::Matrix3d K = camera.getK();
    K /= K(2,2);

    Eigen::Matrix3d KnewInv = Knew.inverse();


    FishEyeCameraMatrix<double> newCamera(Knew, camera.getR(), camera.getT(), camera.getXi());


    CudaFishEyeImageProcessorDeviceCode::fishEyeImageProcessorUndistort(inputImg, outputImage, k1, k2, p1, p2,
                                                                        K(0,0), K(0,2), K(1,1), K(1,2), KnewInv(0,0), KnewInv(0,2), KnewInv(1,1), KnewInv(1,2));

    return std::make_pair<DeviceImage, FishEyeCameraMatrix<double> >((DeviceImage)outputImage, (FishEyeCameraMatrix<double>)newCamera);
}

DeviceImage CudaFishEyeImageProcessor::extractPinhole(double iScale, Eigen::Matrix3d& KPinhole, double k1, double k2, double p1, double p2)
{
    if (inputImg.getNumChannels() != 1)
    {
        PSL_THROW_EXCEPTION("Only grayscale supported.")
    }

    DeviceImage outputImage;
    int width = (int) round(iScale*inputImg.getWidth());
    int height = (int) round(iScale*inputImg.getHeight());

    outputImage.allocatePitched(width, height, 1);

    KPinhole /= KPinhole(2,2);

    if (KPinhole(0,1) != 0)
    {
        PSL_THROW_EXCEPTION("Only Ks without skew allowed.")
    }

    Eigen::Matrix3d KInvPinhole = KPinhole.inverse();

    Eigen::Matrix3d K = camera.getK();
    K /= K(2,2);

    CudaFishEyeImageProcessorDeviceCode::fishEyeImageProcessorUndistortRectify(inputImg, outputImage, camera.getXi(), k1, k2, p1, p2,
                                                                               K(0,0), K(0,2), K(1,1), K(1,2), KInvPinhole(0,0), KInvPinhole(0,2), KInvPinhole(1,1), KInvPinhole(1,2));

    return outputImage;
}

