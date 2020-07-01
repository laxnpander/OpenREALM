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

#ifndef FISHEYEDPETHMAP_H
#define FISHEYEDPETHMAP_H

#include <psl/fishEyeCameraMatrix.h>
#include <string>
#include <fstream>
#include <opencv2/core.hpp>
#include <boost/shared_array.hpp>

namespace PSL
{
    template<typename T, typename U>
    class FishEyeDepthMap
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FishEyeDepthMap();
        FishEyeDepthMap(unsigned int width, unsigned int height, const FishEyeCameraMatrix<U>& cam);

        T& operator() (int x, int y);
        const T& operator() (int x, int y) const;

        T* getDataPtr();

        unsigned int getWidth() const;
        unsigned int getHeight() const;
        const FishEyeCameraMatrix<U>& getCam() const;
        void setCam(const FishEyeCameraMatrix<U>& cam);

        void display(T minZ, T maxZ, int displayTime = 0, const char* windowName = "depth map");
        void displayColored(T minZ, T maxZ, int displayTime = 0, const char* windowName = "depth map colored");
        void displayInvDepth(T minZ, T maxZ, int displayTime = 0, const char* windowName = "inverse depth map");
        void displayInvDepthColored(T minZ, T maxZ, int displayTime = 0, const char* windowName = "color coded inverse depth map");

        void saveInvDepthAsColorImage(std::string fileName, T minZ, T maxZ);

        // returns a point with homogeneous component 0 if depth is invalid
        Eigen::Matrix<U, 4, 1> unproject(int x, int y) const;

        void meshToVRML(std::ofstream& os, std::string textureImageFileName, float scale, float maxDispDiff, U maxDist = -1);
        void pointCloudToVRML(std::ofstream& os, U maxDist = -1);
        void pointCloudColoredToVRML(std::ofstream& os, cv::Mat image, U maxDist = -1);

    private:
        boost::shared_array<T> depths;
        unsigned int width;
        unsigned int height;
        FishEyeCameraMatrix<U> cam;

    };
}

#endif // FISHEYEDEPTHMAP_H
