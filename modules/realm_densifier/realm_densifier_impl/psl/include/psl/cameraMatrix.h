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

#ifndef CAMERAMATRIX_H
#define CAMERAMATRIX_H

#include <eigen3/Eigen/Dense>
#include <string>

namespace PSL
{
    template <typename NumericType>
    class CameraMatrix
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CameraMatrix();
        CameraMatrix(const Eigen::Matrix<NumericType, 3, 4>& P);
        CameraMatrix(const Eigen::Matrix<NumericType, 3, 3>& K, const Eigen::Matrix<NumericType, 3, 3>& R, const Eigen::Matrix<NumericType, 3, 1>& T);

        // copy consturctor and assignment
        CameraMatrix(const CameraMatrix<NumericType>& otherCameraMatrix);
        CameraMatrix& operator=(const CameraMatrix<NumericType>& otherCameraMatrix);

        void scaleK(NumericType scale_x, NumericType scale_y);

        void setKRT(const Eigen::Matrix<NumericType, 3, 3>& K, const Eigen::Matrix<NumericType, 3, 3>& R, const Eigen::Matrix<NumericType, 3, 1>& T);
        void setP(const Eigen::Matrix<NumericType, 3, 4>& P);
        void setRT(const Eigen::Matrix<NumericType, 3, 4>& RT);

        const Eigen::Matrix<NumericType, 3, 3>& getK() const;
        const Eigen::Matrix<NumericType, 3, 3>& getR() const;
        const Eigen::Matrix<NumericType, 3, 1>& getT() const;
        const Eigen::Matrix<NumericType, 3, 4>& getP() const;
        const Eigen::Matrix<NumericType, 3, 1>& getC() const;
        const Eigen::Matrix<NumericType, 4, 4>& getCam2Global() const;

        Eigen::Matrix<NumericType, 4, 1> unprojectPoint(NumericType x, NumericType y, NumericType depth) const;

        Eigen::Matrix<NumericType, 4, 1> localPoint2GlobalPoint(Eigen::Matrix<NumericType, 4, 1>& localPoint) const;

        void loadFromDepthMapDataFile(std::string fileName);

    private:
        Eigen::Matrix<NumericType, 3, 3> K;
        Eigen::Matrix<NumericType, 3, 3> R;
        Eigen::Matrix<NumericType, 3, 1> T;

        void recomputeStoredValues();

        // some prestored values to return
        Eigen::Matrix<NumericType, 3, 4> P;
        Eigen::Matrix<NumericType, 4, 4> cam2Global;
        Eigen::Matrix<NumericType, 3, 1> C;
    };
}


#endif // CAMERAMATRIX_H
