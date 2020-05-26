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

#ifndef CUDACOMMON_H
#define CUDACOMMON_H

#include <sstream>

#include <psl/exception.h>

using std::ostringstream;


namespace PSL_CUDA
{

    const int TILE_WIDTH = 16;
    const int TILE_HEIGHT = 16;

    inline int getNumTiles(int totalSize, int tileSize)
    {
        const int div = totalSize/tileSize;
        return totalSize % tileSize == 0 ? div : div + 1;
    }

#ifdef _MSC_VER

    #define PSL_CUDA_CHECKED_CALL(cuda_call)                                                    \
    {                                                                                       \
        cudaError err = cuda_call;                                                          \
        if( cudaSuccess != err)                                                             \
        {                                                                                   \
            /* generate message */                                                          \
            ostringstream os;                                                               \
            os << "Cuda Error: " << cudaGetErrorString(err);                                \
            throw PSL::Exception(__FILE__, __LINE__, __FUNCSIG__, os.str().c_str());     \
        }                                                                                   \
    }

    #define PSL_CUDA_CHECK_ERROR                                                                \
    {                                                                                       \
        cudaError err = cudaGetLastError();                                                 \
        if( cudaSuccess != err)                                                             \
        {                                                                                   \
            /* generate message */                                                          \
            ostringstream os;                                                               \
            os << "Cuda Error: " << cudaGetErrorString(err);                                \
            throw PSL::Exception(__FILE__, __LINE__, __FUNCSIG__, os.str().c_str());     \
        }                                                                                   \
    }

#else
    #define PSL_CUDA_CHECKED_CALL(cuda_call)                                                    \
    {                                                                                       \
        cudaError err = cuda_call;                                                          \
        if( cudaSuccess != err)                                                             \
        {                                                                                   \
            /* generate message */                                                          \
            ostringstream os;                                                               \
            os << "Cuda Error: " << cudaGetErrorString(err);                                \
            throw PSL::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, os.str().c_str());     \
        }                                                                                   \
    }

    #define PSL_CUDA_CHECK_ERROR                                                                \
    {                                                                                       \
        cudaError err = cudaGetLastError();                                                 \
        if( cudaSuccess != err)                                                             \
        {                                                                                   \
            /* generate message */                                                          \
            ostringstream os;                                                               \
            os << "Cuda Error: " << cudaGetErrorString(err);                                \
            throw PSL::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, os.str().c_str());     \
        }                                                                                   \
    }
#endif
}


#endif //CUDACOMMON_H
