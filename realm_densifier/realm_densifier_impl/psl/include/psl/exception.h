/// This file is part of PlaneSweepLib (PSL)

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

#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
#include <string>
#include <sstream>

using std::stringstream;

namespace PSL
{

#ifdef _MSC_VER
    #define PSL_THROW_EXCEPTION(message) throw PSL::Exception(__FUNCSIG__, (message));
#else
    #define PSL_THROW_EXCEPTION(message) throw PSL::Exception(__PRETTY_FUNCTION__, (message));
#endif

    class Exception : public std::exception
    {
    public:
        Exception(std::string& message)
            : message(message)
        { }

        Exception(const char* place, const char* message)
        {
            stringstream messageStream;
            messageStream  << place << " : " << message;
            this->message = messageStream.str();
        }

        Exception(const char* fileName, int lineNumber, const char* place, const char* message)
        {
            stringstream messageStream;
            messageStream << fileName << "(" << lineNumber << ") " << place << " : " << message;
            this->message = messageStream.str();
        }

        virtual ~Exception() throw() { }

        virtual const char* what() const throw()
        {
            return message.c_str();
        }

    protected:
        std::string message;
    };
}

#endif // EXCEPTION_H
