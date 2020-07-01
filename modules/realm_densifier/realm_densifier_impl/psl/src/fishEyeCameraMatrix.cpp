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

#include <psl/fishEyeCameraMatrix.h>

using namespace PSL;
using Eigen::Matrix;

template<typename NumericType>
FishEyeCameraMatrix<NumericType>::FishEyeCameraMatrix()
{
    xi = 1;
}


template<typename NumericType>
FishEyeCameraMatrix<NumericType>::FishEyeCameraMatrix(const Eigen::Matrix<NumericType, 3, 3>& K, const Eigen::Matrix<NumericType, 3, 3>& R,
                                                      const Eigen::Matrix<NumericType, 3, 1>& T, NumericType xi)
{
    cam.setKRT(K, R, T);

    this->xi = xi;
}

template<typename NumericType>
void FishEyeCameraMatrix<NumericType>::scaleK(NumericType scale_x, NumericType scale_y)
{
    cam.scaleK(scale_x, scale_y);
}

template<typename NumericType>
const Eigen::Matrix<NumericType, 3, 3>& FishEyeCameraMatrix<NumericType>::getK() const
{
    return cam.getK();
}

template<typename NumericType>
const Eigen::Matrix<NumericType, 3, 3>& FishEyeCameraMatrix<NumericType>::getR() const
{
    return cam.getR();
}

template<typename NumericType>
const Eigen::Matrix<NumericType, 3, 1>& FishEyeCameraMatrix<NumericType>::getT() const
{
    return cam.getT();
}

template<typename NumericType>
const NumericType& FishEyeCameraMatrix<NumericType>::getXi() const
{
    return this->xi;
}

template<typename NumericType>
void FishEyeCameraMatrix<NumericType>::setKRTXi(const Eigen::Matrix<NumericType, 3, 3>& K, const Eigen::Matrix<NumericType, 3, 3>& R, const Eigen::Matrix<NumericType, 3, 1>& T, NumericType xi)
{
    this->cam.setKRT(K, R, T);
    this->xi = xi;

}

// current version only unprojects points which are less than 180°
template<typename NumericType>
Eigen::Matrix<NumericType, 4, 1> FishEyeCameraMatrix<NumericType>::unprojectPoint(NumericType x, NumericType y, NumericType depth) const
{
    NumericType mx = (x - cam.getK()(0,2))/cam.getK()(0,0);
    NumericType my = (y - cam.getK()(1,2))/cam.getK()(1,1);

    NumericType mxMySqr = mx*mx + my*my;
    NumericType D = 1+(1-xi*xi)*mxMySqr;
    if (D > 1e-5)
    {
        NumericType fact = (xi + sqrt(D))/(mxMySqr + 1);
        if (fact - xi < 0.1) // if the points go to close to 180° things go crazy!
            return Eigen::Matrix<NumericType, 4, 1>::Zero();

        NumericType fact2 = depth/(fact - xi);

        Eigen::Matrix<NumericType, 4, 1> point;
        point(0) = fact*fact2*mx;
        point(1) = fact*fact2*my;
        point(2) = depth;
        point(3) = 1;

        return cam.localPoint2GlobalPoint(point);

    }

    return Eigen::Matrix<NumericType, 4, 1>::Zero();
}

template<typename NumericType>
Eigen::Matrix<NumericType, 2, 1> FishEyeCameraMatrix<NumericType>::projectPoint(NumericType x, NumericType y, NumericType z) const
{
    Eigen::Matrix<NumericType, 3, 1> point;
    point << x, y, z;

    point = cam.getR()*point;
    point = point + cam.getT();

    NumericType length = point.norm();
    point /= length;

    point(2) += getXi();
    point /= point(2);

    Eigen::Matrix<NumericType, 2, 1> projPoint;
    projPoint = (cam.getK()*point).topRows(2);

    return projPoint;
}

template<typename NumericType>
const Eigen::Matrix<NumericType, 3, 1>& FishEyeCameraMatrix<NumericType>::getC() const
{
    return cam.getC();
}



template class FishEyeCameraMatrix<float>;
template class FishEyeCameraMatrix<double>;
