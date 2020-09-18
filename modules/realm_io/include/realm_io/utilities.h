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

#ifndef PROJECT_UTILITIES_H
#define PROJECT_UTILITIES_H

#include <iostream>
#include <chrono>
#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>

namespace realm
{
namespace io
{

/*!
 * @brief Function that checks if a file exists or not
 * @param filename absolute path to file
 * @return true if exists, false if not
 */
bool fileExists(const std::string &filepath);

/*!
 * @brief Function that checks if a directory exists or not
 * @param directory path to directory
 * @return true if exists, false if not
 */
bool dirExists(const std::string &directory);

/*!
 * @brief Function to create directory
 * @param directory absolute path to directory
 * @throws runtime_error throws error if directory could not be created
 */
void createDir(const std::string &directory);

/*!
 * @brief Function for creation of unique filename based on a frame id
 * @param prefix Prefix of the file, e.g. image_
 * @param frame_id Frame id or unique id for the file, e.g. 213 -> will be formattet as 000213
 * @param suffix File suffix, e.g. .png, .jpg, ...
 * @return constructed string
 */
std::string createFilename(const std::string &prefix, uint32_t frame_id, const std::string &suffix);

/*!
 * @brief Function for determining wether a filename path is given relative or absolute and then returning the absolute
 * path
 * @param directory Directory within the file may have been, e.g. /home/user/example/
 * @param filename Filename of the object, e.g. text.txt, but also possible /home/user/example2/text2.txt
 * @return returns the absolute path if a file is found relative or absolute to the given informations
 */
std::string getAbsolutePath(const std::string &directory, const std::string &filename);

/*!
 * @brief Function to create a string of local date and time
 * @return string in format: year_month_day_hours_minutes_seconds
 */
std::string getDateTime();

/*!
 * @brief Function to get current time in nanoseconds
 * @return Current time since epoch in nanoseconds
 */
uint64_t getCurrentTimeNano();

/*!
 * @brief Extracts the frame id from a file path, e.g. /home/user/img_000015.jpeg -> returns 15
 * @param filepath Filepath
 * @return Frame id
 */
uint32_t extractFrameIdFromFilepath(const std::string &filepath);

/*!
 * @brief Simple function for splitting an input string into several tokens
 * @param str Input string to be splitted
 * @param c Deliminiter that seperates the tokens
 * @return Vector of tokens
 */
std::vector<std::string> split(const char *str, char c = ' ');

/*!
 * @brief Function grabs the files in a specific directory, sorts them and return their absolute path as a vector
 * @param dir Directory to grab the filenames
 * @return Vector of all files with absolute path
 */
std::vector<std::string> getFileList(const std::string& dir);

/*! TODO: Einbaurichtung implementieren?
 * @brief Function to compute a 3x3 rotation matrix based on heading data. It is assumed, that the camera is pointing
 * downwards and the heading roughly aligns with the camera's yaw axis.
 * @param heading Magnetic heading of the camera / UAV
 * @return 3x3 rotation matrix
 */
cv::Mat computeOrientationFromHeading(double heading);

}
}

#endif //PROJECT_UTILITIES_H
