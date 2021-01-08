

#include <realm_io/gdal_continuous_writer.h>

using namespace realm;

io::GDALContinuousWriter::GDALContinuousWriter(const std::string &thread_name, int64_t sleep_time, bool verbose)
 : WorkerThreadBase(thread_name, sleep_time, verbose),
   m_queue_size(1)
{
}

void io::GDALContinuousWriter::requestSaveGeoTIFF(const CvGridMap::Ptr &map,
                                                  const uint8_t &zone,
                                                  const std::string &filename,
                                                  bool do_build_overview,
                                                  bool do_split_save,
                                                  io::GDALProfile gdal_profile)
{
  // Create new save job
  QueueElement::Ptr queue_element;
  queue_element.reset(new QueueElement{map, zone, filename, do_build_overview, do_split_save, gdal_profile});

  // Push it to the processing queue
  m_mutex_save_requests.lock();
  m_save_requests.push_back(queue_element);
  if (m_save_requests.size() > m_queue_size)
  {
    m_save_requests.pop_front();
  }
  m_mutex_save_requests.unlock();
}

bool io::GDALContinuousWriter::process()
{
  m_mutex_save_requests.lock();
  if (!m_save_requests.empty())
  {
    QueueElement::Ptr queue_element = m_save_requests.back();
    m_save_requests.pop_back();
    m_mutex_save_requests.unlock();

    io::saveGeoTIFF(
      *queue_element->map,
      queue_element->zone,
      queue_element->filename,
      queue_element->do_build_overview,
      queue_element->do_split_save,
      queue_element->gdal_profile
    );

    return true;
  }

  m_mutex_save_requests.unlock();
  return false;
}

void io::GDALContinuousWriter::reset()
{

}

void io::GDALContinuousWriter::finishCallback()
{
}