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

#include <psl/deviceImage.h>
#include <psl/deviceImage.cuh>

namespace PSL_CUDA
{
    namespace DeviceImageDeviceCode
    {
        __global__ void clearKernel(DeviceImage devImg, unsigned char value)
        {
            // get position of outupt
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < devImg.getWidth()*devImg.getNumChannels() && y < devImg.getHeight())
            {
                devImg(x,y) = value;
            }
        }
    }
}

using namespace PSL;
using namespace PSL_CUDA;
using namespace DeviceImageDeviceCode;


DeviceImage::DeviceImage()
{
    addr = 0;
}



void DeviceImage::deallocate()
{
    PSL_CUDA_CHECKED_CALL( cudaFree((void *)addr); )
    addr = 0;
}

void DeviceImage::clear(unsigned char value)
{

    dim3 gridDim(getNumTiles(width*numChannels, TILE_WIDTH), getNumTiles(height, TILE_HEIGHT));
    dim3 blockDim(TILE_WIDTH, TILE_HEIGHT);

    clearKernel<<<gridDim, blockDim>>>(*this, value);
}

