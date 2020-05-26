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

#ifndef DEPTHMAP_H
#define DEPTHMAP_H

#include <psl/cameraMatrix.h>
#include <boost/shared_array.hpp>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Eigen>
#include <opencv2/core.hpp>

namespace PSL
{

    using boost::shared_array;
    using std::ofstream;
    using std::string;

    template<typename T, typename U>
    class DepthMap
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        DepthMap();
        DepthMap(unsigned int width, unsigned int height, const CameraMatrix<U>& cam);
        void initialize(T value);
        DepthMap& operator*= (T scale);
        T* getDataPtr();
        unsigned int getWidth() const;
        unsigned int getHeight() const;
        T& operator() (int x, int y);
        const T& operator() (int x, int y) const;
        const CameraMatrix<U>& getCam() const;
        void setCam(const CameraMatrix<U>& new_cam);

        void display(T minZ, T maxZ, int displayTime = 0, const char* windowName = "depth map");
        void displayInvDepth(T minZ, T maxZ, int displayTime = 0, const char* windowName = "inverse depth map");
        void displayInvDepthColored(T minZ, T maxZ, int displayTime = 0, const char* windowName = "color coded inverse depth map");

        // returns a point with homogeneous component 0 if depth is invalid
        Eigen::Matrix<U, 4, 1> unproject(int x, int y) const;

        void meshToVRML(ofstream& os, string textureImageFileName, float scale, float maxDispDiff, U maxDist = -1);
        void meshToVRML(ofstream& os, U maxDist = -1, T maxDispDiff = -1);
        void pointCloudToVRML(ofstream& os, U maxDist = -1);
        void pointCloudColoredToVRML(ofstream& os, cv::Mat image, U maxDist = -1);
        void saveAsImage(string fileName, T minDepth, T maxDepth);
        void saveInvDepthAsImage(string fileName, T minDepth, T maxDepth);
        void saveInvDepthAsColorImage(string fileName, T minZ, T maxZ);

        void saveAsDataFile(string fileName);
        void loadFromDataFile(string fileName);
        void setRT(const Eigen::Matrix<U, 3, 4>& RT);

        cv::Mat convertToCvPclMono(U maxDist = -1);
        void convertToCvPclRGB(const cv::Mat &img, cv::Mat &points, cv::Mat &color);

    private:       
        shared_array<T> depths;
        unsigned int width;
        unsigned int height;
        CameraMatrix<U> cam;
    };
}

#endif // DEPTHMAP_H
