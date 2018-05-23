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

#ifndef DEVICEBUFFER_H
#define DEVICEBUFFER_H

#include "cudaCommon.h"
#include "cuda_runtime.h"
#include <string>


namespace PSL_CUDA
{
    template<typename T>
    class DeviceBuffer
    {
    public:
        DeviceBuffer();

        // host and device functions
        inline __host__ __device__ T* getAddr() const
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
        inline __device__ T& operator()(unsigned int x, unsigned int y);

        // host only functions
        void allocatePitched(int width, int height);
        void reallocatePitched(int width, int height);
        void clear(T value);
        void download(T* dstPtr, size_t dstPitch);
        void downloadAndDisplay(int waitTime, T minVal, T maxVal, std::string windowTitle = "Buffer");
        void upload(T* dataPtr, size_t dataPitch);

        void deallocate();

    private:
        T* addr;
        int width;
        int height;
        size_t pitch;
    };
}

#endif // CUDADEVICEBUFFER_H
