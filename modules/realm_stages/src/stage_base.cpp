

#include <realm_core/loguru.h>

#include <realm_stages/stage_base.h>

using namespace realm;

StageBase::StageBase(const std::string &name, const std::string &path, double rate, int queue_size, bool log_to_file)
: WorkerThreadBase("Stage [" + name + "]", static_cast<int64_t>(1/rate*1000.0), true),
  m_stage_name(name),
  m_stage_path(path),
  m_queue_size(queue_size),
  m_log_to_file(log_to_file),
  m_is_output_dir_initialized(false),
  m_t_statistics_period(10),
  m_counter_frames_in(0),
  m_counter_frames_out(0),
  m_timer_statistics_fps(new Timer(std::chrono::seconds(m_t_statistics_period), std::bind(&StageBase::evaluateStatistic, this)))
{
}

bool StageBase::changeParam(const std::string &name, const std::string &val)
{
  LOG_F(WARNING, "Changing parameter not implemented for this stage!");
  return false;
}

void StageBase::initStagePath(const std::string &abs_path)
{
  // Set and create output directory
  m_stage_path = abs_path + "/" + m_stage_name;
  initStageCallback();
  m_is_output_dir_initialized = true;

  // Init logging if enabled
  if (m_log_to_file)
  {
    loguru::add_file((m_stage_path + "/stage.log").c_str(), loguru::Append, loguru::Verbosity_MAX);
  }

  LOG_F(INFO, "Successfully initialized!");
  LOG_F(INFO, "Stage path set to: %s", m_stage_path.c_str());
  printSettingsToLog();
}

StageStatistics StageBase::getStageStatistics()
{
  std::unique_lock<std::mutex> lock(m_mutex_statistics);
  m_stage_statistics.queue_depth = getQueueDepth();
  return m_stage_statistics;
}

void StageBase::registerAsyncDataReadyFunctor(const std::function<bool()> &func)
{
  // Return true if either the functor evaluates to true, or when a finish is requested.
  m_data_ready_functor = ([=]{ return (func() || isFinishRequested()); });
}

void StageBase::registerFrameTransport(const std::function<void(const Frame::Ptr&, const std::string&)> &func)
{
  m_transport_frame = func;
}

void StageBase::registerPoseTransport(const std::function<void(const cv::Mat &, uint8_t zone, char band, const std::string &)> &func)
{
  m_transport_pose = func;
}

void StageBase::registerDepthMapTransport(const std::function<void(const cv::Mat&, const std::string&)> &func)
{
  m_transport_depth_map = func;
}

void StageBase::registerPointCloudTransport(const std::function<void(const PointCloud::Ptr &, const std::string&)> &func)
{
  m_transport_pointcloud = func;
}

void StageBase::registerImageTransport(const std::function<void(const cv::Mat&, const std::string&)> &func)
{
  m_transport_img = func;
}

void StageBase::registerMeshTransport(const std::function<void(const std::vector<Face>&, const std::string&)> &func)
{
  m_transport_mesh = func;
}

void StageBase::registerCvGridMapTransport(const std::function<void(const CvGridMap &, uint8_t zone, char band, const std::string&)> &func)
{
  m_transport_cvgridmap = func;
}

void StageBase::setStatisticsPeriod(uint32_t s)
{
  std::unique_lock<std::mutex> lock(m_mutex_statistics);
  m_t_statistics_period = s;
}

void StageBase::logCurrentStatistics() const
{
  LOG_SCOPE_F(INFO, "Stage [%s] statistics", m_stage_name.c_str());
  LOG_F(INFO, "Total frames: %i ", m_stage_statistics.frames_total);
  LOG_F(INFO, "Processed frames: %i ", m_stage_statistics.frames_processed);
  LOG_F(INFO, "Bad frames: %i ", m_stage_statistics.frames_bad);
  LOG_F(INFO, "Dropped frames: %i ", m_stage_statistics.frames_dropped);
  LOG_F(INFO, "Fps in: %4.2f ", m_stage_statistics.fps_in);
  LOG_F(INFO, "Fps out: %4.2f ", m_stage_statistics.fps_out);
}

void StageBase::updateStatisticsIncoming()
{
  std::unique_lock<std::mutex> lock(m_mutex_statistics);
  m_counter_frames_in++;
  m_stage_statistics.frames_total++;
}

void StageBase::updateStatisticsSkippedFrame()
{
  std::unique_lock<std::mutex> lock(m_mutex_statistics);
  m_stage_statistics.frames_dropped++;
}

void StageBase::updateStatisticsBadFrame()
{
  std::unique_lock<std::mutex> lock(m_mutex_statistics);
  m_stage_statistics.frames_bad++;
}

void StageBase::updateStatisticsProcessedFrame()
{
  std::unique_lock<std::mutex> lock(m_mutex_statistics);
  m_stage_statistics.frames_processed++;
}

void StageBase::updateStatisticsOutgoing()
{
    std::unique_lock<std::mutex> lock(m_mutex_statistics);
    m_counter_frames_out++;
    m_stage_statistics.process_statistics = getProcessingStatistics();

    uint32_t queue_depth = getQueueDepth();
    if (m_stage_statistics.queue_statistics.count == 0) {
      m_stage_statistics.queue_statistics.min = queue_depth;
      m_stage_statistics.queue_statistics.max = queue_depth;
    } else {
      if (m_stage_statistics.queue_statistics.min > queue_depth) m_stage_statistics.queue_statistics.min = queue_depth;
      if (m_stage_statistics.queue_statistics.max > queue_depth) m_stage_statistics.queue_statistics.max = queue_depth;
    }
    m_stage_statistics.queue_statistics.count++;
    m_stage_statistics.queue_statistics.avg =
        m_stage_statistics.queue_statistics.avg + ((double)queue_depth - m_stage_statistics.queue_statistics.avg)
                                                  / (double)m_stage_statistics.queue_statistics.count;
}

void StageBase::evaluateStatistic()
{
  std::unique_lock<std::mutex> lock(m_mutex_statistics);
  float fps_in = static_cast<float>(m_counter_frames_in) / m_t_statistics_period;
  float fps_out = static_cast<float>(m_counter_frames_out) / m_t_statistics_period;

  m_counter_frames_in = 0;
  m_counter_frames_out = 0;

  logCurrentStatistics();

  m_stage_statistics.fps_in = fps_in;
  m_stage_statistics.fps_out = fps_out;
}