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
#include <sstream>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include <psl/configFile.h>
#include <psl/exception.h>

using namespace PSL;
using std::ifstream;
using std::getline;
using std::istringstream;
using boost::algorithm::trim;

ConfigFile::ConfigFile(const string& fileName) {

    fileRead = false;

    ifstream file;
    file.open(fileName.c_str());

    if (!file.is_open())
        return;

    while (!file.eof()) {
        string line;
        getline(file, line);
        parseLine(line);
    }

    fileRead = true;
}

void ConfigFile::parseLine(string& line) {

    int commentPos = (int) line.find('#');

    // nothing interesting before the comment
    if (commentPos != -1 && commentPos < 3)
        return;

    // remove commented part
    if (commentPos != -1)
    {
        line = line.substr(0, commentPos);
    }
    int eqPos = (int) line.find('=');
    if (eqPos == -1)
        return;

    string before = line.substr(0, eqPos);
    string after = line.substr(eqPos+1, line.length()-eqPos-1);

    trim(before);
    trim(after);

    // if both strings are not emtpy add the pair to the hash table
    if (before.length() > 0 && after.length() > 0)
        this->configEntries[before] = after;
}

string ConfigFile::get(const string& configParameter)
{
    if (configEntries.count(configParameter) != 0)
    {
        return configEntries[configParameter];
    }
    else
    {
        return string();
    }
}

int ConfigFile::getAsInt(const string& configParameter)
{
    std::string paramStr = get(configParameter);
    if (paramStr.empty())
    {
        std::stringstream strStr;
        strStr << "Parameter " << configParameter << " is not defined in config file.";
        PSL_THROW_EXCEPTION(strStr.str().c_str());
    }
    return atoi(paramStr.c_str());
}

float ConfigFile::getAsFloat(const string& configParameter)
{
    std::string paramStr = get(configParameter);
    if (paramStr.empty())
    {
        std::stringstream strStr;
        strStr << "Parameter " << configParameter << " is not defined in config file.";
        PSL_THROW_EXCEPTION(strStr.str().c_str());
    }
    return (float) atof(paramStr.c_str());
}
bool ConfigFile::isFileRead() {
    return fileRead;
}
