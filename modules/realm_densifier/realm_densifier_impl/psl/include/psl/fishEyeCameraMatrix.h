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

#ifndef FISHEYECAMERAMATRIX_H
#define FISHEYECAMERAMATRIX_H

#include <eigen3/Eigen/Eigen>
#include <string>
#include "cameraMatrix.h"

namespace PSL
{
    template <typename NumericType>
    class FishEyeCameraMatrix
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FishEyeCameraMatrix();
        FishEyeCameraMatrix(const Eigen::Matrix<NumericType, 3, 3>& K, const Eigen::Matrix<NumericType, 3, 3>& R, const Eigen::Matrix<NumericType, 3, 1>& T, NumericType xi);

        const NumericType& getXi() const;
        void scaleK(NumericType scale_x, NumericType scale_y);
        void setKRTXi(const Eigen::Matrix<NumericType, 3, 3>& K, const Eigen::Matrix<NumericType, 3, 3>& R, const Eigen::Matrix<NumericType, 3, 1>& T, NumericType xi);

        const Eigen::Matrix<NumericType, 3, 3>& getK() const;
        const Eigen::Matrix<NumericType, 3, 3>& getR() const;
        const Eigen::Matrix<NumericType, 3, 1>& getT() const;
        const Eigen::Matrix<NumericType, 3, 1>& getC() const;

        Eigen::Matrix<NumericType, 4, 1> unprojectPoint(NumericType x, NumericType y, NumericType depth) const;
        Eigen::Matrix<NumericType, 2, 1> projectPoint(NumericType x, NumericType y, NumericType z) const;

    private:
        CameraMatrix<NumericType> cam;
        NumericType xi;
    };
}

#endif // FISHEYECAMERAMATRIX_H
