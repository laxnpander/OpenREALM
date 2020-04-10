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

#include <string>

#include <opencv2/highgui/highgui.hpp>

#include <psl/fishEyeDepthMap.h>
#include <psl/common.h>
#include <psl/colorMapJet.h>
#include <psl/grid.h>

using namespace PSL;
using namespace std;

template <typename T, typename U>
FishEyeDepthMap<T, U>::FishEyeDepthMap()
{
    width = 0;
    height = 0;
}

template <typename T, typename U>
FishEyeDepthMap<T, U>::FishEyeDepthMap(unsigned int width, unsigned int height, const FishEyeCameraMatrix<U> &cam)
{
    this->width = width;
    this->height = height;
    this->cam = cam;

    T* depths = new T[width*height];
    this->depths.reset(depths);
}

template <typename T, typename U>
T* FishEyeDepthMap<T, U>::getDataPtr()
{
    return &(depths[0]);
}

template <typename T, typename U>
unsigned int FishEyeDepthMap<T, U>::getWidth() const
{
    return width;
}

template <typename T, typename U>
unsigned int FishEyeDepthMap<T, U>::getHeight() const
{
    return height;
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::display(T minZ, T maxZ, int displayTime, const char *windowName)
{
    cv::Mat_<T> depthsMat(height, width, getDataPtr());
    cv::imshow(windowName, (depthsMat-minZ)/(maxZ-minZ));
    cv::waitKey(displayTime);
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::displayColored(T minZ, T maxZ, int displayTime, const char *windowName)
{

    cv::Mat colDepth = cv::Mat::zeros(height, width, CV_8UC3);

     for (int y = 0; y < colDepth.rows; y++)
     {
         unsigned char* pixel = colDepth.ptr<unsigned char>(y);
         for (int x = 0; x < colDepth.cols; ++x)
         {
             const T depth = (*this)(x,y);
             if (depth > 0)
             {
                 int idx = (int) round(std::max((T) 0, std::min((depth - minZ)/(maxZ-minZ), (T) 1) * (T) 255));

                 pixel[0] = (unsigned char) floor(colorMapJet[idx][2] * 255.0f + 0.5f);
                 pixel[1] = (unsigned char) floor(colorMapJet[idx][1] * 255.0f + 0.5f);
                 pixel[2] = (unsigned char) floor(colorMapJet[idx][0] * 255.0f + 0.5f);
             }

             pixel += 3;
         }
     }
    cv::imshow(windowName, colDepth);
    cv::waitKey(displayTime);
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::displayInvDepth(T minZ, T maxZ, int displayTime, const char *windowName)
{
    cv::Mat_<T> depthsMat(height, width, getDataPtr());
    cv::Mat_<T> invDepthsMat(height, width);
    for (unsigned int y = 0; y < height; y++)
    {
        for (unsigned int x = 0; x < width; x++)
        {
            const T depth = depthsMat[y][x];
            if (depth > 0)
            {
                invDepthsMat[y][x] = (1/depthsMat[y][x]-1/maxZ)/(1/minZ - 1/maxZ);
            }
            else
            {
                invDepthsMat[y][x] = 0;
            }
        }
    }
    cv::imshow(windowName, invDepthsMat);
    cv::waitKey(displayTime);
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::displayInvDepthColored(T minZ, T maxZ, int displayTime, const char *windowName)
{
    cv::Mat colInvDepth = cv::Mat::zeros(height, width, CV_8UC3);

     for (int y = 0; y < colInvDepth.rows; y++)
     {
         unsigned char* pixel = colInvDepth.ptr<unsigned char>(y);
         for (int x = 0; x < colInvDepth.cols; ++x)
         {
             const T depth = (*this)(x,y);
             if (depth > 0)
             {
                 int idx = (int) round(std::max((T) 0, std::min(1/depth - 1/maxZ, 1/minZ - 1/maxZ) / (1/minZ - 1/maxZ)) * (T) 255);

                 pixel[0] = (int) round(colorMapJet[idx][2] * 255.0f);
                 pixel[1] = (int) round(colorMapJet[idx][1] * 255.0f);
                 pixel[2] = (int) round(colorMapJet[idx][0] * 255.0f);
             }

             pixel += 3;
         }
     }
    cv::imshow(windowName, colInvDepth);
    cv::waitKey(displayTime);
}


template <typename T, typename U>
T& FishEyeDepthMap<T, U>::operator() (int x, int y)
{
    return depths[y*width + x];
}

template <typename T, typename U>
const T& FishEyeDepthMap<T, U>::operator() (int x, int y) const
{
    return depths[y*width + x];
}

template <typename T, typename U>
const FishEyeCameraMatrix<U>& FishEyeDepthMap<T,U>::getCam() const
{
    return cam;
}

template <typename T, typename U>
Eigen::Matrix<U, 4, 1> FishEyeDepthMap<T, U>::unproject(int x, int y) const
{
    U depth = (U) (*this)(x, y);
    if (depth <= 0)
    {
        return Eigen::Matrix<U, 4, 1>::Zero();
    }

    return getCam().unprojectPoint((U) x, (U) y, (U) depth);
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::pointCloudToVRML(std::ofstream& os, U maxDist)
{
    if (!os.is_open())
    {
        PSL_THROW_EXCEPTION("Outputstream not open.")
    }

    PSL::Grid<Eigen::Matrix<U, 4, 1> > reprojPoints(width, height);
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
        {
            reprojPoints(x,y) = unproject(x,y);
            if (maxDist > 0 && reprojPoints(x,y)(3) == 1)
            {
                U dist = (reprojPoints(x,y).topRows(3) - cam.getC()).norm();
                if (maxDist < dist)
                {
                    reprojPoints(x,y)(0) = 0;
                    reprojPoints(x,y)(1) = 0;
                    reprojPoints(x,y)(2) = 0;
                    reprojPoints(x,y)(3) = 0;
                }
            }
        }

    os << "#VRML V2.0 utf8" << std::endl;
    os << "Shape {" << std::endl;
    os << "     appearance Appearance {" << std::endl;
    os << "         material Material { " << std::endl;
    os << "             diffuseColor     0.5 0.5 0.5" << std::endl;
    os << "         }" << std::endl;
    os << "     }" << std::endl;
    os << "     geometry PointSet {" << std::endl;
    os << "       coord Coordinate {" << std::endl;
    os << "           point [" << std::endl;
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
            os << "               " << reprojPoints(x,y)(0) << " " << reprojPoints(x,y)(1) << " " << reprojPoints(x,y)(2) << "," << std::endl;
    os << "           ]" << std::endl;
    os << "       }" << std::endl;
    os << "   }" << std::endl;
    os << "}" << std::endl;
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::pointCloudColoredToVRML(std::ofstream& os, cv::Mat image, U maxDist)
{
    if (!os.is_open())
    {
        PSL_THROW_EXCEPTION("Outputstream not open.")
    }

    Grid<Eigen::Matrix<U, 4, 1> > reprojPoints(width, height);
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
        {
            reprojPoints(x,y) = unproject(x,y);
            if (maxDist > 0 && reprojPoints(x,y)(3) == 1)
            {
                U dist = (reprojPoints(x,y).topRows(3) - cam.getC()).norm();
                if (maxDist < dist)
                {
                    reprojPoints(x,y)(0) = 0;
                    reprojPoints(x,y)(1) = 0;
                    reprojPoints(x,y)(2) = 0;
                    reprojPoints(x,y)(3) = 0;
                }
            }
        }

    os << "#VRML V2.0 utf8" << std::endl;
    os << "Shape {" <<  std::endl;
    os << "     appearance Appearance {" <<  std::endl;
    os << "         material Material { " <<  std::endl;
    os << "             diffuseColor     0.5 0.5 0.5" <<  std::endl;
    os << "         }" <<  std::endl;
    os << "     }" <<  std::endl;
    os << "     geometry PointSet {" <<  std::endl;
    os << "       coord Coordinate {" <<  std::endl;
    os << "           point [" <<  std::endl;
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
            os << "               " << reprojPoints(x,y)(0) << " " << reprojPoints(x,y)(1) << " " << reprojPoints(x,y)(2) << "," <<  std::endl;
    os << "           ]" <<  std::endl;
    os << "       }" <<  std::endl;
    os << "       color Color {" <<  std::endl;
    os << "         color [" <<  std::endl;
    if (image.channels() == 1)
    {
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
        {
            os << "           " << image.at<unsigned char>(y,x)/255.0 << " " << image.at<unsigned char>(y,x)/255.0 << " " << image.at<unsigned char>(y,x)/255.0 << "," <<  std::endl;
        }
    }
    else if (image.channels() == 3)
    {
        for (unsigned int y = 0; y < height; y++)
            for (unsigned int x = 0; x < width; x++)
            {
                os << "           " << image.at<unsigned char>(y,3*x+2)/255.0 << " " << image.at<unsigned char>(y,3*x+1)/255.0 << " " << image.at<unsigned char>(y,3*x)/255.0 << "," <<  std::endl;
            }
    }
    os << "         ]" <<  std::endl;
    os << "       }" <<  std::endl;
    os << "   }" <<  std::endl;
    os << "}" <<  std::endl;
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::setCam(const FishEyeCameraMatrix<U>& cam)
{
    this->cam = cam;
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::saveInvDepthAsColorImage(std::string fileName, T minZ, T maxZ)
{

   cv::Mat colDepth = cv::Mat::zeros(height, width, CV_8UC3);

    for (int y = 0; y < colDepth.rows; y++)
    {
        unsigned char* pixel = colDepth.ptr<unsigned char>(y);
        for (int x = 0; x < colDepth.cols; ++x)
        {
            const T depth = (*this)(x,y);
            if (depth > 0)
            {
                int idx = (int) round(std::max((T) 0, std::min(1/depth - 1/maxZ, 1/minZ - 1/maxZ) / (1/minZ - 1/maxZ)) * (T) 255);

                pixel[0] = (int) round(colorMapJet[idx][2] * 255.0f);
                pixel[1] = (int) round(colorMapJet[idx][1] * 255.0f);
                pixel[2] = (int) round(colorMapJet[idx][0] * 255.0f);
            }

            pixel += 3;
        }
    }

    cv::imwrite(fileName.c_str(), colDepth);
}

template <typename T, typename U>
void FishEyeDepthMap<T, U>::meshToVRML(std::ofstream& os, std::string textureImageFileName, float scale, float maxDispDiff, U maxDist)
{
    if (!os.is_open())
    {
        PSL_THROW_EXCEPTION("Outputstream not open.")
    }

    Grid<Eigen::Matrix<U, 4, 1> > reprojPoints(width, height);
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
        {
            reprojPoints(x,y) = unproject(x,y);

            if (maxDist > 0 && reprojPoints(x,y)(3) == 1)
            {
                U dist = (reprojPoints(x,y).topRows(3) - cam.getC()).norm();
                if (maxDist < dist)
                {
                    reprojPoints(x,y)(3) = 0;
                }
            }
        }

    os << "#VRML V2.0 utf8" << endl;
    os << "Shape {" << endl;
    os << "     appearance Appearance {" << endl;
    os << "         texture ImageTexture {" << endl;
    os << "             url [\"" << textureImageFileName << "\"]" << endl;
    os << "         }" << endl;
    os << "         material Material { " << endl;
    os << "             diffuseColor     0.5 0.5 0.5" << endl;
    os << "         }" << endl;
    os << "     }" << endl;
    os << "     geometry IndexedFaceSet {" << endl;
    os << "       coord Coordinate {" << endl;
    os << "           point [" << endl;
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
            os << "               " << reprojPoints(x,y)(0) << " " << reprojPoints(x,y)(1) << " " << reprojPoints(x,y)(2) << "," << endl;
    os << "           ]" << endl;
    os << "       }" << endl;
    os << "       coordIndex [" << endl;
    for (unsigned int y = 0; y < height-1; y++)
        for (unsigned int x = 0; x < width-1; x++)
        {
            float dispDiff = (float) std::max(std::fabs(scale/(*this)(x,y) - scale/(*this)(x+1,y)) , std::max(std::fabs(scale/(*this)(x+1,y)-scale/(*this)(x,y+1)), std::fabs(scale/(*this)(x,y+1))- scale/(*this)(x,y)));
            if ((maxDispDiff < 0 || dispDiff < maxDispDiff) &&  reprojPoints(x,y)(3) == 1 && reprojPoints(x+1,y)(3) == 1 && reprojPoints(x,y+1)(3) == 1)
                os << "           " << y*width + x +1 << ", " << y*width + x << ", " << (y+1)*width + x << ", -1," << endl;
            dispDiff = (float) std::max(std::fabs(scale/(*this)(x+1,y+1)-scale/(*this)(x+1,y)), std::max(std::fabs(scale/(*this)(x+1,y)-scale/(*this)(x,y+1)), std::fabs(scale/(*this)(x,y+1))-scale/(*this)(x+1,y+1)));
            if ((maxDispDiff < 0 || dispDiff < maxDispDiff) && reprojPoints(x+1,y+1)(3) == 1 && reprojPoints(x+1,y)(3) == 1 && reprojPoints(x,y+1)(3) == 1)
                os << "           " << (y+1)*width + x + 1 << ", " <<  y*width + x +1 << ", " << (y+1)*width + x  << ", -1," << endl;
        }
    os << "       ]" << endl;
    os << "       texCoord TextureCoordinate {" << endl;
    os << "         point [" << endl;
    for (unsigned int y = 0; y < height; y++)
        for (unsigned int x = 0; x < width; x++)
            os << "                 " << (float) x/(float)width << " " << height - (float)y/(float)height << ", " << endl;
    os << "         ]" << endl;
    os << "     }" << endl;
    os << "   }" << endl;
    os << "}" << endl;
}





// instantiate
template class FishEyeDepthMap<float, float>;
template class FishEyeDepthMap<float, double>;
template class FishEyeDepthMap<double, float>;
template class FishEyeDepthMap<double, double>;
