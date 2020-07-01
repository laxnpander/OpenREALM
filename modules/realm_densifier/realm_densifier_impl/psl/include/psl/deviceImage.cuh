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

#ifndef DEVICEIMAGE_CUH
#define DEVICEIMAGE_CUH

inline __device__ unsigned char& PSL_CUDA::DeviceImage::operator()(unsigned int x, unsigned int y)
{
    return *(addr + y*pitch + x);
}

inline __device__ unsigned char& PSL_CUDA::DeviceImage::operator()(unsigned int x, unsigned int y, unsigned int c)
{
    return *(addr + y*pitch + x*numChannels + c);
}


#endif // DEVICEIMAGE_CUH
