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
#include <opencv2/highgui/highgui.hpp>

#include <psl/deviceBuffer.h>

using namespace PSL_CUDA;

template <typename T>
void DeviceBuffer<T>::downloadAndDisplay(int waitTime, T minVal, T maxVal, std::string windowTitle)
{
    cv::Mat_<T> mat(height, width);

    download((T*)mat.data, mat.step);

    mat -= minVal;
    mat /= (maxVal - minVal);

    cv::imshow(windowTitle.c_str(), mat);

    cv::waitKey(waitTime);
}

template<typename T>
void DeviceBuffer<T>::download(T* dstPtr, size_t dstPitch)
{
    PSL_CUDA_CHECKED_CALL( cudaMemcpy2D(dstPtr, dstPitch, addr, pitch, sizeof(T)*width, height, cudaMemcpyDeviceToHost); )
}

#ifdef _MSC_VER
#pragma warning( disable : 4661)
#endif

template class DeviceBuffer<float>;
template class DeviceBuffer<int>;
