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

#include <cmath>
#include <iostream>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <psl/depthMap.h>
#include <psl/exception.h>
#include <psl/grid.h>
#include <psl/ioTools.h>
#include <psl/colorMapJet.h>
#include <psl/common.h>

using std::endl;
using Eigen::Matrix;

namespace PSL
{
    template <typename T, typename U>
    DepthMap<T, U>::DepthMap()
    {
        width = 0;
        height = 0;
    }

    template <typename T, typename U>
    DepthMap<T, U>::DepthMap(unsigned int width, unsigned int height, const CameraMatrix<U> &cam)
    {
        this->width = width;
        this->height = height;
        this->cam = cam;

        T* depths = new T[width*height];
        this->depths.reset(depths);
    }
    template<typename T, typename U>
    void DepthMap<T, U>::initialize(T value)
    {
        for(unsigned int i=0; i<width*height; ++i)
            depths[i]=value;
    }

    template <typename T, typename U>
    T* DepthMap<T, U>::getDataPtr()
    {
        return &(depths[0]);
    }

    template <typename T, typename U>
    T& DepthMap<T, U>::operator() (int x, int y)
    {
        return depths[y*width + x];
    }

    template <typename T, typename U>
    const T& DepthMap<T, U>::operator() (int x, int y) const
    {
        return depths[y*width + x];
    }

    template <typename T, typename U>
    unsigned int DepthMap<T, U>::getWidth() const
    {
        return width;
    }

    template <typename T, typename U>
    unsigned int DepthMap<T, U>::getHeight() const
    {
        return height;
    }

    template <typename T, typename U>
    void DepthMap<T, U>::display(T minZ, T maxZ, int displayTime, const char *windowName)
    {
        cv::Mat_<T> depthsMat(height, width, getDataPtr());
        cv::imshow(windowName, (depthsMat-minZ)/(maxZ-minZ));
        cv::waitKey(displayTime);
    }

    template <typename T, typename U>
    void DepthMap<T, U>::displayInvDepth(T minZ, T maxZ, int displayTime, const char *windowName)
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
    void DepthMap<T, U>::displayInvDepthColored(T minZ, T maxZ, int displayTime, const char *windowName)
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
    const CameraMatrix<U>& DepthMap<T, U>::getCam() const
    {
        return cam;
    }

    template <typename T, typename U>
    void DepthMap<T, U>::setCam(const CameraMatrix<U>& new_cam)
    {
        cam=new_cam;
    }

    template <typename T, typename U>
    Matrix<U, 4, 1> DepthMap<T, U>::unproject(int x, int y) const
    {
        U depth = (U) (*this)(x,y);
        if (depth <= 0)
        {
            return Matrix<U, 4, 1>::Zero();
        }

        return getCam().unprojectPoint((U)x, (U)y, depth);
    }

    template <typename T, typename U>
    DepthMap<T, U>& DepthMap<T, U>::operator*=(T scale)
    {
        for (unsigned int y = 0; y < getHeight(); y++)
            for (unsigned int x = 0; x < getWidth(); x++)
            {
                if((*this)(x,y)>0)
                    (*this)(x,y) *= scale;
            }
        return (*this);
    }

    template <typename T, typename U>
    void DepthMap<T, U>::meshToVRML(ofstream& os, string textureImageFileName, float scale, float maxDispDiff, U maxDist)
    {
        if (!os.is_open())
        {
            PSL_THROW_EXCEPTION("Outputstream not open.")
        }

        Grid<Matrix<U, 4, 1> > reprojPoints(width, height);
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

    template <typename T, typename U>
    void DepthMap<T, U>::meshToVRML(ofstream& os, U maxDist, T maxDispDiff)
    {
        if (!os.is_open())
        {
            PSL_THROW_EXCEPTION("Outputstream not open.")
        }

        Grid<Matrix<U, 4, 1> > reprojPoints(width, height);
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
                float dispDiff = (float) std::max(std::fabs(1.0/(*this)(x,y) - 1.0/(*this)(x+1,y)) , std::max(std::fabs(1.0/(*this)(x+1,y)-1.0/(*this)(x,y+1)), std::fabs(1.0/(*this)(x,y+1))- 1.0/(*this)(x,y)));
                if ((maxDispDiff < 0 || dispDiff < maxDispDiff) && reprojPoints(x,y)(3) == 1 && reprojPoints(x+1,y)(3) == 1 && reprojPoints(x,y+1)(3) == 1)
                    os << "           " << y*width + x +1 << ", " << y*width + x << ", " << (y+1)*width + x << ", -1," << endl;

                dispDiff = (float) std::max(std::fabs(1.0/(*this)(x,y) - 1.0/(*this)(x+1,y)) , std::max(std::fabs(1.0/(*this)(x+1,y)-1.0/(*this)(x,y+1)), std::fabs(1.0/(*this)(x,y+1))- 1.0/(*this)(x,y)));
                if ((maxDispDiff < 0 || dispDiff < maxDispDiff) && reprojPoints(x+1,y+1)(3) == 1 && reprojPoints(x+1,y)(3) == 1 && reprojPoints(x,y+1)(3) == 1)
                    os << "           " << (y+1)*width + x + 1 << ", " <<  y*width + x +1 << ", " << (y+1)*width + x  << ", -1," << endl;
            }
        os << "       ]" << endl;
        os << "   }" << endl;
        os << "}" << endl;
    }

    template <typename T, typename U>
    void DepthMap<T, U>::pointCloudToVRML(ofstream& os, U maxDist)
    {
        if (!os.is_open())
        {
            PSL_THROW_EXCEPTION("Outputstream not open.")
        }

        Grid<Matrix<U, 4, 1> > reprojPoints(width, height);
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

        os << "#VRML V2.0 utf8" << endl;
        os << "Shape {" << endl;
        os << "     appearance Appearance {" << endl;
        os << "         material Material { " << endl;
        os << "             diffuseColor     0.5 0.5 0.5" << endl;
        os << "         }" << endl;
        os << "     }" << endl;
        os << "     geometry PointSet {" << endl;
        os << "       coord Coordinate {" << endl;
        os << "           point [" << endl;
        for (unsigned int y = 0; y < height; y++)
            for (unsigned int x = 0; x < width; x++)
                os << "               " << reprojPoints(x,y)(0) << " " << reprojPoints(x,y)(1) << " " << reprojPoints(x,y)(2) << "," << endl;
        os << "           ]" << endl;
        os << "       }" << endl;
        os << "   }" << endl;
        os << "}" << endl;
    }

    template <typename T, typename U>
    void DepthMap<T, U>::saveAsImage(string fileName, T minDepth, T maxDepth)
    {
        cv::Mat_<T> sliceMat(this->getHeight(), this->getWidth(), depths.get());
        cv::Mat_<T> sliceMat2 = sliceMat - minDepth;
        cv::imwrite(fileName, sliceMat2/(maxDepth-minDepth)*255);
    }

    template <typename T, typename U>
    void DepthMap<T, U>::saveInvDepthAsImage(string fileName, T minDepth, T maxDepth)
    {
        cv::Mat_<T> invDepthsMat(height, width);
        for (unsigned int y = 0; y < height; y++)
        {
            for (unsigned int x = 0; x < width; x++)
            {
                const T depth = (*this)(x,y);
                if (depth > 0)
                {
                    invDepthsMat[y][x] = (1/(*this)(x,y)-1/maxDepth)/(1/minDepth - 1/maxDepth);
                }
                else
                {
                    invDepthsMat[y][x] = 0;
                }
            }
        }
        cv::imwrite(fileName.c_str(), invDepthsMat*255);
    }

    template <typename T, typename U>
    void DepthMap<T, U>::saveInvDepthAsColorImage(string fileName, T minZ, T maxZ)
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
    void DepthMap<T, U>::saveAsDataFile(string fileName)
    {

        if (CHAR_BIT != 8)
        {
            PSL_THROW_EXCEPTION("Only platforms with 8 bit chars are supported.")
        }

        std::ofstream outStream;
        outStream.open(fileName.c_str(), std::ios::out | std::ios::binary);

        if (!outStream.is_open())
        {
            PSL_THROW_EXCEPTION("Could not open depth map data output file.")
        }

        // file format version, might be useful at some point
        unsigned char version = 1;
        outStream.write((char*)&version, 1);

        // endianness
        unsigned char endian = is_little_endian() ? 0 : 1;
        outStream.write((char*)&endian, 1);

        // store sizes of data types written
        // first unsigned int in an unsigned char because we know that char has always size 1
        unsigned char uintSize = sizeof(unsigned int);
        outStream.write((char*)&uintSize, 1);

        // for T and U we use unsigned int
        unsigned int tSize = sizeof(T);
        outStream.write((char*)&tSize, sizeof(unsigned int));
        unsigned int uSize = sizeof(U);
        outStream.write((char*)&uSize, sizeof(unsigned int));

        // first store camera matrix
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 4; j++)
                outStream.write((char*)&(cam.getP()(i,j)), sizeof(U));

        // now we store the size of the depth map
        outStream.write((char*)&width, sizeof(unsigned int));
        outStream.write((char*)&height, sizeof(unsigned int));
        outStream.write((char*)getDataPtr(), sizeof(T)*width*height);

        if (!outStream.good())
        {
            PSL_THROW_EXCEPTION("An error occured while writing out a depth map to a data file.")
        }

        // writing is done closing stream
        outStream.close();
    }

    template <typename T, typename U>
    void DepthMap<T, U>::loadFromDataFile(string fileName)
    {
        if (CHAR_BIT != 8)
        {
            PSL_THROW_EXCEPTION("Only platforms with 8 bit chars are supported.")
        }

        std::ifstream inStream;
        inStream.open(fileName.c_str(), std::ios::in | std::ios::binary);

        if (!inStream.is_open())
        {
            PSL_THROW_EXCEPTION("Could not open depth map data input file.")
        }

        // read in version
        unsigned char version;
        inStream.read((char*)&version, 1);
        if (version != 1)
        {
            PSL_THROW_EXCEPTION("Only version 1 is supported.")
        }

        // read in endian
        unsigned char endian;
        inStream.read((char*)&endian, 1);

        unsigned char currentEndian = is_little_endian() ? 0: 1;
        if (endian != currentEndian)
        {
            PSL_THROW_EXCEPTION("Current platform does not have the same endian as the depht map data file.")
        }

        // read in the size of an unsigned int from file
        unsigned char uintSize;
        inStream.read((char*)&uintSize, 1);

        // check if current plattform has the same unsigned int size
        if (uintSize != sizeof (unsigned int))
        {
            PSL_THROW_EXCEPTION("Current platform does not have the same unsigned int size as the one the file was written with.")
        }

        unsigned int tSize;
        unsigned int uSize;
        inStream.read((char*)&tSize, sizeof(unsigned int));
        inStream.read((char*)&uSize, sizeof(unsigned int));
        if (tSize != sizeof(T) || uSize != sizeof(U))
        {
            PSL_THROW_EXCEPTION("Sizes of the datatypes for sparse and dense geometry do not match.")
        }

        // read camera matrix
        Eigen::Matrix<U, 3, 4> P;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 4; j++)
                inStream.read((char*)&(P(i,j)), sizeof(U));

        cam.setP(P);

        // read the size of the depth map
        inStream.read((char*)&width, sizeof(unsigned int));
        inStream.read((char*)&height, sizeof(unsigned int));

        // set the shared array to the right size
        T* depths = new T[width*height];
        this->depths.reset(depths);

        // read in the depth data
        inStream.read((char*)getDataPtr(), sizeof(T)*width*height);

        // check if we have read the depth map correctly
        if (!inStream.good())
        {
            PSL_THROW_EXCEPTION("Error reading in the depth map data file.")
        }

        // colse the stream
        inStream.close();
    }
    template <typename T, typename U>
    void DepthMap<T, U>::setRT(const Eigen::Matrix<U, 3, 4>& RT)
    {
        cam.setRT(RT);
    }


    template <typename T, typename U>
    void DepthMap<T, U>::pointCloudColoredToVRML(ofstream& os, cv::Mat image, U maxDist)
    {
        if (!os.is_open())
        {
            PSL_THROW_EXCEPTION("Outputstream not open.")
        }

        Grid<Matrix<U, 4, 1> > reprojPoints(width, height);
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

        os << "#VRML V2.0 utf8" << endl;
        os << "Shape {" << endl;
        os << "     appearance Appearance {" << endl;
        os << "         material Material { " << endl;
        os << "             diffuseColor     0.5 0.5 0.5" << endl;
        os << "         }" << endl;
        os << "     }" << endl;
        os << "     geometry PointSet {" << endl;
        os << "       coord Coordinate {" << endl;
        os << "           point [" << endl;
        for (unsigned int y = 0; y < height; y++)
            for (unsigned int x = 0; x < width; x++)
                os << "               " << reprojPoints(x,y)(0) << " " << reprojPoints(x,y)(1) << " " << reprojPoints(x,y)(2) << "," << endl;
        os << "           ]" << endl;
        os << "       }" << endl;
        os << "       color Color {" << endl;
        os << "         color [" << endl;
        if (image.channels() == 1)
        {
        for (unsigned int y = 0; y < height; y++)
            for (unsigned int x = 0; x < width; x++)
            {
                os << "           " << image.at<unsigned char>(y,x)/255.0 << " " << image.at<unsigned char>(y,x)/255.0 << " " << image.at<unsigned char>(y,x)/255.0 << "," << endl;
            }
        }
        else if (image.channels() == 3)
        {
            for (unsigned int y = 0; y < height; y++)
                for (unsigned int x = 0; x < width; x++)
                {
                    os << "           " << image.at<unsigned char>(y,3*x+2)/255.0 << " " << image.at<unsigned char>(y,3*x+1)/255.0 << " " << image.at<unsigned char>(y,3*x)/255.0 << "," << endl;
                }
        }
        os << "         ]" << endl;
        os << "       }" << endl;
        os << "   }" << endl;
        os << "}" << endl;
    }

    template <typename T, typename U>
    cv::Mat DepthMap<T, U>::convertToCvPclMono(U maxDist)
    {
        Grid<Matrix<U, 4, 1> > reprojPoints(width, height);
        for (unsigned int y = 0; y < height; y++)
            for (unsigned int x = 0; x < width; x++)
            {
                reprojPoints(x, y) = unproject(x, y);

                if (maxDist > 0 && reprojPoints(x, y)(3) == 1)
                {
                    U dist = (reprojPoints(x, y).topRows(3) - cam.getC()).norm();
                    if (maxDist < dist)
                    {
                        reprojPoints(x, y)(0) = 0;
                        reprojPoints(x, y)(1) = 0;
                        reprojPoints(x, y)(2) = 0;
                        reprojPoints(x, y)(3) = 0;
                    }
                }
            }
        cv::Mat pcl(height, width, cv::DataType<U>::type);
        for (unsigned int y = 0; y < height; y++)
            for (unsigned int x = 0; x < width; x++)
            {
              if (!(reprojPoints(x, y)(0) < 0.1
                    && reprojPoints(x, y)(1) < 0.1
                    && reprojPoints(x, y)(2) < 0.1
                    && reprojPoints(x, y)(3) < 0.1))
              {
                cv::Vec<U, 3> p(reprojPoints(x,y)(0), reprojPoints(x,y)(1), reprojPoints(x,y)(2));
                pcl.at<cv::Vec<U, 3>>(y, x) = p;
              }
            }
        // TODO: Not elegant
        if (pcl.type() != CV_64F)
            pcl.convertTo(pcl, CV_64F);
        return pcl;
    }

    template <typename T, typename U>
    void DepthMap<T, U>::convertToCvPclRGB(const cv::Mat &img, cv::Mat &points_out, cv::Mat &color_out)
    {
        U maxDist = -1; // use basic min/max filter
        Grid<Matrix<U, 4, 1> > reprojPoints(width, height);
        for (unsigned int y = 0; y < height; y++)
            for (unsigned int x = 0; x < width; x++)
            {
                reprojPoints(x, y) = unproject(x, y);

                if (maxDist > 0 && reprojPoints(x, y)(3) == 1)
                {
                    U dist = (reprojPoints(x, y).topRows(3) - cam.getC()).norm();
                    if (maxDist < dist)
                    {
                        reprojPoints(x, y)(0) = 0;
                        reprojPoints(x, y)(1) = 0;
                        reprojPoints(x, y)(2) = 0;
                        reprojPoints(x, y)(3) = 0;
                    }
                }
            }

        // Save point coordinates
        cv::Mat points;
        points.reserve(width*height);
        cv::Mat color;
        color.reserve(width*height);
        for (unsigned int y = 0; y < height; y++)
            for (unsigned int x = 0; x < width; x++)
            {
                if (!(reprojPoints(x, y)(0) < 0.1
                    && reprojPoints(x, y)(1) < 0.1
                    && reprojPoints(x, y)(2) < 0.1
                    && reprojPoints(x, y)(3) < 0.1))
                {
                    cv::Mat pt = (cv::Mat_<U>(1, 3) << reprojPoints(x,y)(0), reprojPoints(x,y)(1), reprojPoints(x,y)(2));
                    points.push_back(pt);
                    if (img.channels() == 1)
                      color.push_back(img.at<uchar>(y, x));
                    else if(img.channels() == 3)
                      color.push_back(img.at<cv::Vec3b>(y, x));
                    else if(img.channels() == 4)
                      color.push_back(img.at<cv::Vec4b>(y, x));
                    else
                        throw(std::invalid_argument("Error: Converting to colored pointcloud failed. Image channel mismatch!"));
                }
            }
        color_out = color;
        // TODO: Not elegant
        if (points.type() != CV_64F)
          points.convertTo(points_out, CV_64F);
        else
          points_out = points;
    };

    // instantiate
    template class DepthMap<float, float>;
    template class DepthMap<float, double>;
    template class DepthMap<double, float>;
    template class DepthMap<double, double>;
}
