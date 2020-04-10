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

#include <fstream>
#include <psl/ioTools.h>
#include <psl/exception.h>

namespace PSL
{
    bool is_little_endian()
    {
        short int word = 1;
        char *byte = (char *) &word;
        return byte[0] != 0;
    }

    std::string extractBaseFileName(const std::string &fullFileName)
    {
        size_t pos = fullFileName.find_last_of('/');
        std::string baseName;
        if (pos != std::string::npos)
        {
            baseName = fullFileName.substr(pos+1, fullFileName.size()-pos);
        }
        else
        {
            baseName = fullFileName;
        }
        // remove the ending
        pos = baseName.find_first_of('.');
        if (pos != std::string::npos)
        {
            return baseName.substr(0, pos);
        }
        else
        {
            return baseName;
        }
    }

    std::string extractFileName(const std::string &fullFileName)
    {
        size_t pos = fullFileName.find_last_of('/');
        if (pos != std::string::npos)
        {
            return fullFileName.substr(pos+1, fullFileName.size()-pos);
        }
        else
        {
            return fullFileName;
        }
    }

    std::string extractPath(const std::string &fullFileName)
    {
        size_t pos = fullFileName.find_last_of('/');
        if (pos != std::string::npos)
        {
            return fullFileName.substr(0, pos+1);
        }
        else
        {
            return "./";
        }
    }


    void readObjFile(const std::string &fileName, std::vector<Eigen::Vector3d>& vertices, std::vector<std::vector<int> >& faces)
    {
        std::ifstream objStream;
        objStream.open(fileName.c_str());

        if (!objStream.is_open())
        {
            PSL_THROW_EXCEPTION("Error opening obj file.")
        }

        std::string line;
        while (!objStream.eof())
        {
            getline(objStream, line);

            int commentPos = (int) line.find('#');

            // nothing interesting before the comment
            if (commentPos != -1 && commentPos < 3)
                continue;

            // remove commented part
            if (commentPos != -1)
            {
                line = line.substr(0, commentPos);
            }

            stringstream lineStream(line);

            std::string firstInLine;
            lineStream >> firstInLine;
            if (firstInLine == "v")
            {
                // now we need to read the three coordinates
                double x, y, z;
                lineStream >> x >> y >> z;
                Eigen::Vector3d point;
                point(0) = x; point(1) = y; point(2) = z;
                vertices.push_back(point);
                //cout << "vertex found: (" << x << ", " << y << ", " << z << ")" << endl;
            }
            else if (firstInLine == "f")
            {
                // we need to read the indices
                std::vector<std::string> indexStrings;
                std::string nextString;
                while (lineStream >> nextString)
                {
                    indexStrings.push_back(nextString);
                }

                std::vector<int> face;

                for (unsigned int i = 0; i < indexStrings.size(); i++)
                {
                    stringstream indexStream(indexStrings[i]);
                    int idx;
                    indexStream >> idx;
                    face.push_back(idx-1);
                    //std::cout << idx-1 << " ";
                }
                //std::cout << endl;

                //cout << "face found: " << endl;

                faces.push_back(face);
            }
        }
    }
}
