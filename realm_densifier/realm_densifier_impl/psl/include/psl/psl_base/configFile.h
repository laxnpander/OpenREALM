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

#ifndef CONFIGFILE_H
#define CONFIGFILE_H

#include <map>
#include <string>

namespace PSL
{
    using std::map;
    using std::string;

    class ConfigFile {
    public:
        ConfigFile(const string& fileName);

        string get(const string& configParameter);
        int getAsInt(const string& configParameter);
        float getAsFloat(const string& configParameter);

        bool isFileRead();

    private:

        bool fileRead;

        void parseLine(string& line);

        map<string, string> configEntries;
    };
}

#endif // CONFIGFILE_H
