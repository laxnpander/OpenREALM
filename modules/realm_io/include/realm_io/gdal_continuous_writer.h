

#ifndef OPENREALM_GDAL_CONTINUOUS_WRITER_H
#define OPENREALM_GDAL_CONTINUOUS_WRITER_H

#include <realm_core/worker_thread_base.h>
#include <realm_io/gis_export.h>

namespace realm
{

namespace io
{

class GDALContinuousWriter : public WorkerThreadBase
{
public:
  using Ptr = std::shared_ptr<GDALContinuousWriter>;

public:
  GDALContinuousWriter(const std::string &thread_name, int64_t sleep_time, bool verbose);

  void requestSaveGeoTIFF(const CvGridMap::Ptr &map,
                          const uint8_t &zone,
                          const std::string &filename,
                          bool do_build_overview = false,
                          bool do_split_save = false,
                          GDALProfile gdal_profile = GDALProfile::COG);

private:
  struct QueueElement
  {
    CvGridMap::Ptr map;
    uint8_t zone;
    std::string filename;
    bool do_build_overview;
    bool do_split_save;
    GDALProfile gdal_profile;

    using Ptr = std::shared_ptr<QueueElement>;
  };

  int m_queue_size;

  std::mutex m_mutex_save_requests;
  std::deque<QueueElement::Ptr> m_save_requests;

  bool process() override;

  void reset() override;

  void finishCallback() override;

};

}

} // namespace realm

#endif //OPENREALM_GDAL_CONTINUOUS_WRITER_H
