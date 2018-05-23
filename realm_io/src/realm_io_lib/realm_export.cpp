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

#include <realm_io/realm_export.h>

namespace realm
{
namespace io
{

void saveTimestamp(uint64_t timestamp,
                   uint32_t frame_id,
                   const std::string &directory,
                   const std::string &name)
{
  std::string filename = (directory + "/" + name + ".txt");
  saveTimestamp(timestamp, frame_id, filename);
}

void saveTimestamp(uint64_t timestamp,
                   uint32_t frame_id,
                   const std::string &filename)
{
// Open file and write data
  std::ofstream file;
  file.open(filename.c_str(), std::ios::app);
  file << frame_id << " " << timestamp << std::endl;
}

void saveTrajectory(uint64_t timestamp,
                    const cv::Mat &pose,
                    const std::string &directory,
                    const std::string &name)
{
  // Grab container data
  std::string filename = (directory + "/" + name + ".txt");
  cv::Mat t = pose.col(3);
  cv::Mat R = pose.rowRange(0, 3).colRange(0, 3);

  Eigen::Matrix3d R_eigen;
  R_eigen << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
  Eigen::Quaterniond q(R_eigen);

  // Open file and write data
  std::ofstream file;
  file.open(filename.c_str(), std::ios::app);
  saveTrajectoryTUM(&file, timestamp, t.at<double>(0), t.at<double>(1), t.at<double>(2), q.x(), q.y(), q.z(), q.w());
  file.close();
}

void saveTrajectoryTUM(std::ofstream *file,
                       uint64_t timestamp,
                       double x,
                       double y,
                       double z,
                       double qx,
                       double qy,
                       double qz,
                       double qw)
{
  // TUM file format
  (*file).precision(10);
  (*file) << timestamp << " " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
}

} // namespace io
} // namespace realm