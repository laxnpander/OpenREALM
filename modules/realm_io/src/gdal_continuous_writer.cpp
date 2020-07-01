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

#include <realm_io/gdal_continuous_writer.h>

using namespace realm;

io::GDALContinuousWriter::GDALContinuousWriter(const std::string &thread_name, int64_t sleep_time, bool verbose)
 : WorkerThreadBase(thread_name, sleep_time, verbose),
   _queue_size(1)
{
}

bool io::GDALContinuousWriter::requestSaveGeoTIFF(const CvGridMap::Ptr &map,
                                                  const std::string &color_layer_name,
                                                  const uint8_t &zone,
                                                  const std::string &filename,
                                                  bool do_build_overview,
                                                  bool do_split_save,
                                                  io::GDALProfile gdal_profile)
{
  // Create new save job
  QueueElement::Ptr queue_element;
  queue_element.reset(new QueueElement{map, color_layer_name, zone, filename, do_build_overview, do_split_save, gdal_profile});

  // Push it to the processing queue
  _mutex_save_requests.lock();
  _save_requests.push_back(queue_element);
  if (_save_requests.size() > _queue_size)
  {
    _save_requests.pop_front();
  }
  _mutex_save_requests.unlock();
}

bool io::GDALContinuousWriter::process()
{
  _mutex_save_requests.lock();
  if (!_save_requests.empty())
  {
    QueueElement::Ptr queue_element = _save_requests.back();
    _save_requests.pop_back();
    _mutex_save_requests.unlock();

    io::saveGeoTIFF(
      *queue_element->map,
      queue_element->color_layer,
      queue_element->zone,
      queue_element->filename,
      queue_element->do_build_overview,
      queue_element->do_split_save,
      queue_element->gdal_profile
    );

    return true;
  }

  _mutex_save_requests.unlock();
  return false;
}

void io::GDALContinuousWriter::reset()
{

}

void io::GDALContinuousWriter::finishCallback()
{
}