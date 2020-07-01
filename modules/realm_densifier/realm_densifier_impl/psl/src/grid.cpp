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

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <psl/grid.h>

namespace PSL
{

    template<typename Elem>
    void saveGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, const char* fileName)
    {
        cv::Mat_<Elem> sliceMat(grid.getHeight(), grid.getWidth(), &grid(0,0,z));

        double min, max;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(sliceMat, &min, &max, &minLoc, &maxLoc);

        cv::Mat_<Elem> sliceMat2 = sliceMat - min;
        cv::imwrite(fileName, sliceMat2/(max-min)*255);
    }

    template<typename Elem>
    void saveGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, Elem minVal, Elem maxVal, const char* fileName)
    {
        cv::Mat_<Elem> sliceMat(grid.getHeight(), grid.getWidth(), &grid(0,0,z));
        cv::Mat_<Elem> sliceMat2 = sliceMat - minVal;
        cv::imwrite(fileName, sliceMat2/(maxVal-minVal)*255);
    }

    template<typename Elem>
    void displayGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, long time, const char* windowName)
    {
        cv::Mat_<Elem> sliceMat(grid.getHeight(), grid.getWidth(), &grid(0,0,z));

        double min, max;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(sliceMat, &min, &max, &minLoc, &maxLoc);

        cv::Mat_<Elem> sliceMat2 = sliceMat - min;
        cv::imshow(windowName, sliceMat2/(max-min));
        cv::waitKey(time);
    }

    template<typename Elem>
    void displayGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, Elem minVal, Elem maxVal,  long time, const char* windowName)
    {
        cv::Mat_<Elem> sliceMat(grid.getHeight(), grid.getWidth(), &grid(0,0,z));
        cv::Mat_<Elem> sliceMat2 = sliceMat - minVal;

        cv::imshow(windowName, sliceMat2/(maxVal-minVal));
        cv::waitKey(time);
    }

    template void saveGridZSliceAsImage<double>(Grid<double>& grid, unsigned int z, const char* fileName);
    template void saveGridZSliceAsImage<double>(Grid<double>& grid, unsigned int z, double minVal, double maxVal, const char* fileName);

    template void saveGridZSliceAsImage<float>(Grid<float>& grid, unsigned int z, const char* fileName);
    template void saveGridZSliceAsImage<float>(Grid<float>& grid, unsigned int z, float minVal, float maxVal, const char* fileName);

    template void displayGridZSliceAsImage<double>(Grid<double>& grid, unsigned int z, long time, const char* windowName);
    template void displayGridZSliceAsImage<double>(Grid<double>& grid, unsigned int z, double minVal, double maxVal, long time, const char* windowName);

    template void displayGridZSliceAsImage<float>(Grid<float>& grid, unsigned int z, long time, const char* windowName);
    template void displayGridZSliceAsImage<float>(Grid<float>& grid, unsigned int z, float minVal, float maxVal, long time, const char* windowName);
}
