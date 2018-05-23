/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <realm_io/utilities.h>

using namespace realm;

bool io::fileExists(const std::string &filepath)
{
  return boost::filesystem::exists(filepath);
}

bool io::dirExists(const std::string &directory)
{
  return boost::filesystem::exists(directory);
}

void io::createDir(const std::string &directory)
{
  if (io::dirExists(directory))
    return;
  boost::filesystem::path dir(directory);
  try
  {
    boost::filesystem::create_directory(dir);
  }
  catch (...)
  {
    // Currently no exception handling provided
  }
}

std::string io::createFilename(const std::string &prefix, uint32_t frame_id, const std::string &suffix)
{
  char filename[1000];
  sprintf(filename, (prefix + "%06i" + suffix).c_str(), frame_id);
  return std::string(filename);
}

std::string io::getAbsolutePath(const std::string &directory, const std::string &filename)
{
  if (io::fileExists(filename))
    return filename;
  if (io::fileExists(directory + filename))
    return (directory + filename);
  throw(std::invalid_argument("Error: File '" + filename + "' could not be found."));
}

std::string io::getDateTime()
{
  time_t     now = time(nullptr);
  tm  tstruct = *localtime(&now);
  char tim[20];
  strftime(tim, sizeof(tim), "%y-%m-%d_%H-%M-%S", &tstruct);
  return std::string(tim);
}

uint64_t io::getCurrentTimeNano()
{
  using namespace std::chrono;
  nanoseconds ms = duration_cast<nanoseconds>(system_clock::now().time_since_epoch());
  return static_cast<uint64_t>(ms.count());
}

uint32_t io::extractFrameIdFromFilepath(const std::string &filepath)
{
  std::vector<std::string> tokens_path = io::split(filepath.c_str(), '/');
  std::vector<std::string> tokens_name = io::split(tokens_path.back().c_str(), '.');
  std::string filename = tokens_name[0];
  return static_cast<uint32_t>(std::stoul(filename.substr(filename.size()-4,filename.size())));
}

std::vector<std::string> io::split(const char *str, char c)
{
  std::vector<std::string> result;

  do
  {
    const char *begin = str;

    while(*str != c && *str)
      str++;

    result.emplace_back(std::string(begin, str));
  }
  while (0 != *str++);

  return result;
}