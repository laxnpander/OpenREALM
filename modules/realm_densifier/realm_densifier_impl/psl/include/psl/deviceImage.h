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

#ifndef DEVICEIMAGE_H
#define DEVICEIMAGE_H

#include "cudaCommon.h"
#include "cuda_runtime.h"

namespace cv
{
 class Mat;
}


namespace PSL_CUDA
{
    class DeviceImage
    {
    public:
        DeviceImage();

        // host and device functions
        inline __host__ __device__ unsigned char* getAddr() const
        {
            return addr;
        }
        inline __host__ __device__ int getWidth() const
        {
            return width;
        }
        inline __host__ __device__ int getHeight() const
        {
            return height;
        }
        inline __host__ __device__ int getNumChannels() const
        {
            return numChannels;
        }
        inline __host__ __device__ int getPitch() const
        {
            return (int) pitch;
        }
        inline __host__ __device__ bool isAllocated() const
        {
            return addr != 0;
        }
        inline __host__ __device__ int2 getSize() const
        {
            return make_int2(width, height);
        }

        // device only functions
        inline __device__ unsigned char& operator()(unsigned int x, unsigned int y);
        inline __device__ unsigned char& operator()(unsigned int x, unsigned int y, unsigned int c);

        // host only functions
        void allocatePitchedAndUpload(const cv::Mat& img);
        void reallocatePitchedAndUpload(const cv::Mat& img);
        void allocatePitched(int width, int height, int numChannels);
        void download(cv::Mat& img);
        void clear(unsigned char value);
        void deallocate();

    private:
        unsigned char* addr;
        int width;
        int height;
        int numChannels;
        size_t pitch;
    };
}

#endif // CUDADEVICEIMAGE_H
