

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
 * @brief Function to remove a file
 * @param filepath Absolute path to the file or directory
 * @return True if succesfull removal
 */
bool removeFileOrDirectory(const std::string &filepath);

/*!
 * @brief Function for creation of unique filename based on a frame id
 * @param prefix Prefix of the file, e.g. image_
 * @param frame_id Frame id or unique id for the file, e.g. 213 -> will be formattet as 000213
 * @param suffix File suffix, e.g. .png, .jpg, ...
 * @return constructed string
 */
std::string createFilename(const std::string &prefix, uint32_t frame_id, const std::string &suffix);

/*!
 * @brief Function for determining the system temporary directory
 * @return System temp directory
 */
std::string getTempDirectoryPath();

/*!
 * @brief Function to create a string of local date and time
 * @return string in format: year_month_day_hours_minutes_seconds
 */
std::string getDateTime();

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
std::vector<std::string> getFileList(const std::string& dir, const std::string &suffix = "");

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
