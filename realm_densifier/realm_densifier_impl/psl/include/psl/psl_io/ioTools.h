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

#ifndef IOTOOLS_H
#define IOTOOLS_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Eigen>


namespace PSL
{
    bool is_little_endian();
    std::string extractBaseFileName(const std::string& fullFileName);
    std::string extractFileName(const std::string& fullFileName);
    std::string extractPath(const std::string& fullFileName);

    // reads a simple obj file vertex indices are returned 0 based
    void readObjFile(const std::string& fileName, std::vector<Eigen::Vector3d>& vertices, std::vector<std::vector<int> >& faces);
}


#endif // IOTOOLS_H
