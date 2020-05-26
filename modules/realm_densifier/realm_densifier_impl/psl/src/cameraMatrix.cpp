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
#include <fstream>
#include <psl/cameraMatrix.h>
#include <psl/exception.h>
#include <psl/ioTools.h>

using namespace PSL;
using Eigen::Matrix;

template<typename NumericType>
CameraMatrix<NumericType>::CameraMatrix()
{
    this->K.setIdentity();
    this->R.setIdentity();
    this->T.setZero();

    recomputeStoredValues();
}

template<typename NumericType>
CameraMatrix<NumericType>::CameraMatrix(const Matrix<NumericType, 3, 3> &K, const Matrix<NumericType, 3, 3> &R, const Matrix<NumericType, 3, 1>& T)
{
    this->K = K;
    this->R = R;
    this->T = T;

    // scale K such that K(2,2) = 1
    NumericType scale = 1/K(2,2);
    this->K *= scale;

    recomputeStoredValues();
}

template<typename NumericType>
CameraMatrix<NumericType>::CameraMatrix(const CameraMatrix<NumericType> &otherCameraMatrix)
{
    this->K = otherCameraMatrix.K;
    this->R = otherCameraMatrix.R;
    this->T = otherCameraMatrix.T;

    recomputeStoredValues();
}

template<typename NumericType>
CameraMatrix<NumericType>& CameraMatrix<NumericType>::operator=(const CameraMatrix<NumericType>& otherCameraMatrix)
{
    this->K = otherCameraMatrix.K;
    this->R = otherCameraMatrix.R;
    this->T = otherCameraMatrix.T;

    recomputeStoredValues();

    return *this;
}

template<typename NumericType>
CameraMatrix<NumericType>::CameraMatrix(const Matrix<NumericType, 3, 4>& P)
{
    setP(P);
}

template<typename NumericType>
void CameraMatrix<NumericType>::setP(const Matrix<NumericType, 3, 4>& P)
{
    K(0,0) = P(0,0); K(0,1) = P(0,1); K(0,2) = P(0,2);
    K(1,0) = P(1,0); K(1,1) = P(1,1); K(1,2) = P(1,2);
    K(2,0) = P(2,0); K(2,1) = P(2,1); K(2,2) = P(2,2);

    // Calculate RQ decomposition of M
    // Hartley & Zissermann, p579 (2nd ed)

    NumericType d = std::sqrt(K(2,2)*K(2,2) + K(2,1)*K(2,1));
    NumericType c = -K(2,2)/d;
    NumericType s = K(2,1)/d;

    Matrix<NumericType, 3, 3> Rx;
    Rx(0,0) = 1; Rx(0,1) = 0; Rx(0,2) = 0;
    Rx(1,0) = 0; Rx(1,1) = c; Rx(1,2) = -s;
    Rx(2,0) = 0; Rx(2,1) = s; Rx(2,2) = c;

    K = K*Rx;

    d = std::sqrt(K(2,2)*K(2,2) + K(2,0)*K(2,0));
    c = K(2,2)/d;
    s = K(2,0)/d;

    Matrix<NumericType, 3, 3> Ry;
    Ry(0,0) = c; Ry(0,1) = 0; Ry(0,2) = s;
    Ry(1,0) = 0; Ry(1,1) = 1; Ry(1,2) = 0;
    Ry(2,0) = -s; Ry(2,1) = 0; Ry(2,2) = c;

    K = K*Ry;

    d = std::sqrt(K(1,1)*K(1,1) + K(1,0)*K(1,0));
    c = -K(1,1)/d;
    s = K(1,0)/d;

    Matrix<NumericType, 3, 3> Rz;
    Rz(0,0) = c; Rz(0,1) = -s; Rz(0,2) = 0;
    Rz(1,0) = s; Rz(1,1) = c; Rz(1,2) = 0;
    Rz(2,0) = 0; Rz(2,1) = 0; Rz(2,2) = 1;

    K = K*Rz;

    Matrix<NumericType, 3, 3> Sign;
    Sign.setIdentity();
    if (K(0,0) < 0) Sign(0,0) = -1;
    if (K(1,1) < 0) Sign(1,1) = -1;
    if (K(2,2) < 0) Sign(2,2) = -1;

    K = K*Sign;

    R = Rx*Ry*Rz*Sign;
    R.transposeInPlace();

    T(0) = P(0,3);
    T(1) = P(1,3);
    T(2) = P(2,3);

    T = K.inverse()*T;

    // scale K such that K(2,2) = 1
    NumericType scale = 1/K(2,2);
    K(0,0) *= scale; K(0,1) *= scale; K(0,2) *= scale;
    K(1,1) *= scale; K(1,2) *= scale;
    K(2,2) *= scale;

    recomputeStoredValues();
}

template<typename NumericType>
const Matrix<NumericType, 3, 3>& CameraMatrix<NumericType>::getK() const
{
    return K;
}

template<typename NumericType>
const Matrix<NumericType, 3, 3>& CameraMatrix<NumericType>::getR() const
{
    return R;
}

template<typename NumericType>
const Matrix<NumericType, 3, 1>& CameraMatrix<NumericType>::getT() const
{
    return T;
}

template<typename NumericType>
const Matrix<NumericType, 3, 1>& CameraMatrix<NumericType>::getC() const
{
    return C;
}

template<typename NumericType>
const Matrix<NumericType, 3, 4>& CameraMatrix<NumericType>::getP() const
{
    return P;
}

template<typename NumericType>
void CameraMatrix<NumericType>::setKRT(const Matrix<NumericType, 3, 3>& K, const Matrix<NumericType, 3, 3>& R, const Matrix<NumericType, 3, 1>& T)
{
    this->K = K;
    this->R = R;
    this->T = T;

    recomputeStoredValues();
}

template<typename NumericType>
void CameraMatrix<NumericType>::setRT(const Matrix<NumericType, 3, 4>& RT)
{
    Matrix<NumericType, 3, 3> R;
    Matrix<NumericType, 3, 1> T;
    R(0,0)=RT(0,0); R(0,1)=RT(0,1); R(0,2)=RT(0,2); T(0)=RT(0,3);
    R(1,0)=RT(1,0); R(1,1)=RT(1,1); R(1,2)=RT(1,2); T(1)=RT(1,3);
    R(2,0)=RT(2,0); R(2,1)=RT(2,1); R(2,2)=RT(2,2); T(2)=RT(2,3);

    this->R = R;
    this->T = T;

    recomputeStoredValues();
}

template<typename NumericType>
const Matrix<NumericType, 4, 4> &CameraMatrix<NumericType>::getCam2Global() const
{
    return cam2Global;
}

template<typename NumericType>
void CameraMatrix<NumericType>::recomputeStoredValues()
{
    // compute P
    P.topLeftCorner(3,3) = R;
    P.rightCols(1) = T;
    P = K*P;

    // compute C
    C = -R.inverse()*T;

    // compute cam2Global
    cam2Global.setIdentity();
    cam2Global.topLeftCorner(3,3) = R.transpose();
    cam2Global.topRightCorner(3,1) = C;
}

template<typename NumericType>
Matrix<NumericType, 4, 1> CameraMatrix<NumericType>::unprojectPoint(NumericType x, NumericType y, NumericType depth) const
{
    Matrix<NumericType, 4, 1> point;

    point(0) = (x - K(0,2))*depth/K(0,0);
    point(1) = (y - K(1,2))*depth/K(1,1);
    point(2) = depth;
    point(3) = 1;

    return cam2Global*point;
}

template<typename NumericType>
void CameraMatrix<NumericType>::scaleK(NumericType scale_x, NumericType scale_y)
{
    K(0,0) *= scale_x;
    K(0,1) *= scale_x;
    K(0,2) = (K(0,2) + (NumericType) 0.5)*scale_x - (NumericType) 0.5;
    K(1,1) *= scale_y;
    K(1,2) = (K(1,2) + (NumericType) 0.5)*scale_y - (NumericType) 0.5;

}

template<typename NumericType>
void CameraMatrix<NumericType>::loadFromDepthMapDataFile(std::string fileName)
{
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
        PSL_THROW_EXCEPTION("Current platform does not have the same endian as the depth map data file.")
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
    if (uSize != sizeof(NumericType))
    {
        PSL_THROW_EXCEPTION("Size of numeric data type does not match.")
    }

    // read camera matrix
    Eigen::Matrix<NumericType, 3, 4> P;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            inStream.read((char*)&(P(i,j)), sizeof(NumericType));

    if (!inStream.good())
    {
        PSL_THROW_EXCEPTION("Error while reading the camera matrix")
    }

    setP(P);
}

template<typename NumericType>
Eigen::Matrix<NumericType, 4, 1> CameraMatrix<NumericType>::localPoint2GlobalPoint(Eigen::Matrix<NumericType, 4, 1>& localPoint) const
{
    return cam2Global*localPoint;
}

template class CameraMatrix<float>;
template class CameraMatrix<double>;
