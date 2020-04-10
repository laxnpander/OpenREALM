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

#ifndef GRID_H
#define GRID_H

#include <boost/shared_array.hpp>
#include <cstdlib>
#include <string>
#include <fstream>
#include <psl/exception.h>
#include <psl/ioTools.h>
#include <climits>

namespace PSL
{
    template <typename Elem>
            class Grid
    {

    public:
        Grid();
        Grid(unsigned int xDim, unsigned int yDim, unsigned int zDim = 1);
        Grid(unsigned int xDim, unsigned int yDim, unsigned int zDim, const Elem& value);

        const Elem& operator()(unsigned int x, unsigned int y, unsigned int z = 0) const;
        Elem& operator()(unsigned int x, unsigned int y, unsigned int z = 0);

        Grid<Elem> clone() const;

        unsigned int getWidth() const;
        unsigned int getHeight() const;
        unsigned int getDepth() const;
        unsigned int getNbVoxels() const;

        Elem* getDataPtr() const;

        void resize(unsigned int _xDim, unsigned int _yDim, unsigned int _zDim);

        void saveAsDataFile(const std::string& fileName);
        void loadFromDataFile(const std::string& fileName);

    protected:        
        boost::shared_array<Elem> _cells;
        unsigned int _xDim, _yDim, _zDim;

        unsigned int _xyDim;
        unsigned int _xyzDim;
    };

    template<typename Elem>
    void saveGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, const char* fileName);

    template<typename Elem>
    void saveGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, Elem minVal, Elem maxVal, const char* fileName);

    template<typename Elem>
    void displayGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, long time=0, const char* windowName = "Grid");

    template<typename Elem>
    void displayGridZSliceAsImage(Grid<Elem>& grid, unsigned int z, Elem minVal, Elem maxVal,  long time=0, const char* windowName = "Grid");
}

template <typename Elem>
PSL::Grid<Elem>::Grid()
{
    _xDim = 0;
    _yDim = 0;
    _zDim = 0;
    _xyDim = 0;
    _xyzDim = 0;
}

template <typename Elem>
PSL::Grid<Elem>::Grid(unsigned int xDim, unsigned int yDim, unsigned int zDim)
{
    this->_xDim = xDim;
    this->_yDim = yDim;
    this->_zDim = zDim;
    this->_xyDim = xDim*yDim;
    this->_xyzDim = _xyDim*zDim;
    Elem* cellsPtr = new Elem[_xyzDim];
    _cells.reset(cellsPtr);
}

template <typename Elem>
PSL::Grid<Elem>::Grid(unsigned int xDim, unsigned int yDim, unsigned int zDim, const Elem &value)
{
    this->_xDim = xDim;
    this->_yDim = yDim;
    this->_zDim = zDim;
    this->_xyDim = xDim*yDim;
    this->_xyzDim = _xyDim*zDim;
    Elem* cellsPtr = new Elem[_xyzDim];
    _cells.reset(cellsPtr);

    for (unsigned int i = 0; i < _xyzDim; i++)
    {
        _cells[i] = value;
    }

}

template <typename Elem>
const Elem& PSL::Grid<Elem>::operator ()(unsigned int x, unsigned int y, unsigned int z) const
{
    return _cells[z*_xyDim + y*_xDim + x];
}

template <typename Elem>
Elem& PSL::Grid<Elem>::operator ()(unsigned int x, unsigned int y, unsigned int z)
{
    return _cells[z*_xyDim + y*_xDim + x];
}

template <typename Elem>
void PSL::Grid<Elem>::resize(unsigned int xDim, unsigned int yDim, unsigned int zDim)
{
    if(xDim == this->_xDim && yDim == this->_yDim && zDim == this->_zDim)
        return;

    this->_xDim = xDim;
    this->_yDim = yDim;
    this->_zDim = zDim;
    this->_xyDim = xDim*yDim;
    this->_xyzDim = _xyDim*zDim;

    _cells = boost::shared_array<Elem>(new Elem[_xyzDim]);
}

template <typename Elem>
unsigned int PSL::Grid<Elem>::getWidth() const
{
    return _xDim;
}

template <typename Elem>
unsigned int PSL::Grid<Elem>::getHeight() const
{
    return _yDim;
}

template <typename Elem>
unsigned int PSL::Grid<Elem>::getDepth() const
{
    return _zDim;
}

template <typename Elem>
unsigned int PSL::Grid<Elem>::getNbVoxels() const
{
    return _xyzDim;
}

template <typename Elem>
Elem* PSL::Grid<Elem>::getDataPtr() const
{
    return _cells.get();
}

template <typename Elem>
PSL::Grid<Elem> PSL::Grid<Elem>::clone() const
{
    Grid<Elem> result(getWidth(), getHeight(), getDepth());
    memcpy((void*)result.getDataPtr(), (void*)this->getDataPtr(), sizeof(Elem)*getNbVoxels());
    return result;
}

template <typename Elem>
void PSL::Grid<Elem>::saveAsDataFile(const std::string& fileName)
{
    if (CHAR_BIT != 8)
    {
        PSL_THROW_EXCEPTION("Only platforms with 8 bit chars are supported.")
    }

    std::ofstream outStream;
    outStream.open(fileName.c_str(), std::ios::out | std::ios::binary);

    if (!outStream.is_open())
    {
        PSL_THROW_EXCEPTION("Could not open grid data output file for writing.")
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

    // for Elem we use unsigned int
    unsigned int elemSize = sizeof(Elem);
    outStream.write((char*)&elemSize, sizeof(unsigned int));

    // now we store the size of the grid
    outStream.write((char*)&_xDim, sizeof(unsigned int));
    outStream.write((char*)&_yDim, sizeof(unsigned int));
    outStream.write((char*)&_zDim, sizeof(unsigned int));

    // now grid data is written
    outStream.write((char*)getDataPtr(), sizeof(Elem)*_xyzDim);

    if (!outStream.good())
    {
        PSL_THROW_EXCEPTION("An error occured while writing the grid to a data file.")
    }

    // writing is done closing stream
    outStream.close();
}

template <typename Elem>
void PSL::Grid<Elem>::loadFromDataFile(const std::string& fileName)
{
    if (CHAR_BIT != 8)
    {
        PSL_THROW_EXCEPTION("Only platforms with 8 bit chars are supported.")
    }

    std::ifstream inStream;
    inStream.open(fileName.c_str(), std::ios::in | std::ios::binary);

    if (!inStream.is_open())
    {
        PSL_THROW_EXCEPTION("Could not open grid data input file.")
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

    unsigned int elemSize;
    inStream.read((char*)&elemSize, sizeof(unsigned int));
    if (elemSize != sizeof(Elem))
    {
        PSL_THROW_EXCEPTION("Size of the datatype stored in the grid does not match with the one from the file.")
    }

    // read the grid size
    unsigned int width, height, depth;
    inStream.read((char*)&width, sizeof(unsigned int));
    inStream.read((char*)&height, sizeof(unsigned int));
    inStream.read((char*)&depth, sizeof(unsigned int));

    // resize the grid
    resize(width, height, depth);

    // load the data stored in the grid
    inStream.read((char*)getDataPtr(), sizeof(Elem)*width*height*depth);

    // check stream
    if (!inStream.good())
    {
        PSL_THROW_EXCEPTION("Error while loading the grid from the data file")
    }

    inStream.close();
}


#endif // GRID_H
