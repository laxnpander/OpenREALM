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

#include <iostream>

#include <opencv2/highgui.hpp>

#include <psl/deviceImage.h>

using namespace PSL_CUDA;

void DeviceImage::allocatePitchedAndUpload(const cv::Mat& img)
{
    if (img.type() == CV_8UC4)
    {
        this->width = img.cols;
        this->height = img.rows;
        this->numChannels = 4;
    }
    else if (img.type() == CV_8UC1)
    {
        this->width = img.cols;
        this->height = img.rows;
        this->numChannels = 1;
    }
    else
    {
        PSL_THROW_EXCEPTION ( "Only BRGA and Grayscale supported!")
    }

    PSL_CUDA_CHECKED_CALL( cudaMallocPitch(&addr, &pitch, width*numChannels, height); )
    PSL_CUDA_CHECKED_CALL( cudaMemcpy2D(addr, pitch, img.data, img.step, width*numChannels, height, cudaMemcpyHostToDevice); )
}

void DeviceImage::reallocatePitchedAndUpload(const cv::Mat& img)
{
    if ((img.type() == CV_8UC4 && numChannels != 4)
            || (img.type() == CV_8UC1 && numChannels != 1)
            || (img.cols != width)
            || (img.rows != height))
    {
        deallocate();
        allocatePitchedAndUpload(img);
    }
    else
    {
        PSL_CUDA_CHECKED_CALL( cudaMemcpy2D(addr, pitch, img.data, img.step, width*numChannels, height, cudaMemcpyHostToDevice); )
    }
}

void DeviceImage::allocatePitched(int width, int height, int numChannels)
{
    this->width = width;
    this->height = height;
    this->numChannels = numChannels;

    PSL_CUDA_CHECKED_CALL( cudaMallocPitch(&addr, &pitch, width*numChannels, height); )
}

void DeviceImage::download(cv::Mat& img)
{
    if (numChannels == 4)
    {
        img = cv::Mat(height, width, CV_8UC4);
    }
    else if (numChannels == 1)
    {
        img = cv::Mat(height, width, CV_8UC1);
    }
    else
    {
        PSL_THROW_EXCEPTION ( "Only BRGA and BRG supported!")
    }
    PSL_CUDA_CHECKED_CALL( cudaMemcpy2D(img.data, img.step, addr, pitch, width*numChannels, height, cudaMemcpyDeviceToHost); )
}
