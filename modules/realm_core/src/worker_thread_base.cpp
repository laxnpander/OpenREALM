

#define LOGURU_IMPLEMENTATION 1
#include <realm_core/loguru.h>

#include <functional>

#include <realm_core/worker_thread_base.h>

using namespace realm;

WorkerThreadBase::WorkerThreadBase(const std::string &thread_name, int64_t sleep_time, bool verbose)
: m_thread_name(thread_name),
  m_sleep_time(sleep_time),
  m_finish_requested(false),
  m_reset_requested(false),
  m_stop_requested(false),
  m_is_stopped(false),
  m_verbose(verbose),
  m_data_ready_functor([=]{ return isFinishRequested(); })
{
  if (m_sleep_time == 0)
    throw(std::runtime_error("Error: Worker thread was created with 0s sleep time."));
}

void WorkerThreadBase::start()
{
  startCallback();
  m_thread = std::thread(std::bind(&WorkerThreadBase::run, this));
}

void WorkerThreadBase::join()
{
  if (m_thread.joinable())
    m_thread.join();
}

void WorkerThreadBase::run()
{
  // To have better readability in the log file we set the thread name
  loguru::set_thread_name(m_thread_name.c_str());

  LOG_IF_F(INFO, m_verbose, "Thread '%s' starting loop...", m_thread_name.c_str());
  bool is_first_run = true;

  while (!isFinishRequested())
  {
    std::unique_lock<std::mutex> lock(m_mutex_processing);
    if (!is_first_run)
      m_condition_processing.wait_for(lock, std::chrono::milliseconds(m_sleep_time), m_data_ready_functor);
    else
      is_first_run = false;

    // Handle stops and finish
    if (isStopRequested())
    {
      LOG_IF_F(INFO, m_verbose, "Thread '%s' stopped!", m_thread_name.c_str());
      while (isStopped() && !isFinishRequested())
      {
        if (isResetRequested())
        {
          reset();
          LOG_IF_F(INFO, m_verbose, "Thread '%s' reset!", m_thread_name.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep_time));
      }
      LOG_IF_F(INFO, m_verbose, "Thread '%s' resumed to loop!", m_thread_name.c_str());
    }

    // Check if reset was requested and execute if necessary
    if (isResetRequested())
    {
      reset();
      LOG_IF_F(INFO, m_verbose, "Thread '%s' reset!", m_thread_name.c_str());
    }

    // Calls to derived classes implementation of process()
    long t = getCurrentTimeMilliseconds();
    if (process())
    {
      LOG_IF_F(INFO,
               m_verbose,
               "Timing [Total]: %lu ms",
               getCurrentTimeMilliseconds() - t);

      // Only Update statistics for processing if we did work
      std::unique_lock<std::mutex> stat_lock(m_mutex_statistics);
      long timing = getCurrentTimeMilliseconds() - t;
      if (m_process_statistics.count == 0) {
        m_process_statistics.min = (double)timing;
        m_process_statistics.max = (double)timing;
      } else {
        if (m_process_statistics.min > (double)timing) m_process_statistics.min = (double)timing;
        if (m_process_statistics.max < (double)timing) m_process_statistics.max = (double)timing;
      }
      m_process_statistics.count++;
      m_process_statistics.avg = m_process_statistics.avg + ((double)timing - m_process_statistics.avg) / (double)m_process_statistics.count;
    }

  }
  LOG_IF_F(INFO, m_verbose, "Thread '%s' finished!", m_thread_name.c_str());
}

void WorkerThreadBase::resume()
{
  std::unique_lock<std::mutex> lock(m_mutex_is_stopped);
  if (m_is_stopped)
    m_is_stopped = false;
}

void WorkerThreadBase::requestStop()
{
  std::unique_lock<std::mutex> lock(m_mutex_stop_requested);
  m_stop_requested = true;
  LOG_IF_F(INFO, m_verbose, "Thread '%s' received stop request...", m_thread_name.c_str());
}

void WorkerThreadBase::requestReset()
{
  std::unique_lock<std::mutex> lock(m_mutex_reset_requested);
  m_reset_requested = true;
  LOG_IF_F(INFO, m_verbose, "Thread '%s' received reset request...", m_thread_name.c_str());
}

void WorkerThreadBase::requestFinish()
{
  std::unique_lock<std::mutex> lock(m_mutex_finish_requested);
  m_finish_requested = true;
  LOG_IF_F(INFO, m_verbose, "Thread '%s' received finish request...", m_thread_name.c_str());
  finishCallback();
}

bool WorkerThreadBase::isStopRequested()
{
  std::unique_lock<std::mutex> lock(m_mutex_stop_requested);
  std::unique_lock<std::mutex> lock1(m_mutex_is_stopped);
  if (m_stop_requested && !m_is_stopped)
  {
    m_is_stopped = true;
  }
  return m_stop_requested;
}

bool WorkerThreadBase::isResetRequested()
{
  std::unique_lock<std::mutex> lock(m_mutex_reset_requested);
  return m_reset_requested;
}

bool WorkerThreadBase::isFinishRequested()
{
  std::unique_lock<std::mutex> lock(m_mutex_finish_requested);
  return m_finish_requested;
}

bool WorkerThreadBase::isStopped()
{
  std::unique_lock<std::mutex> lock(m_mutex_is_stopped);
  if (m_is_stopped)
    m_stop_requested = false;
  return m_is_stopped;
}

void WorkerThreadBase::notify()
{
  m_condition_processing.notify_one();
}

Statistics WorkerThreadBase::getProcessingStatistics() {
  std::unique_lock<std::mutex> lock(m_mutex_statistics);
  return m_process_statistics;
}

long WorkerThreadBase::getCurrentTimeMilliseconds()
{
  using namespace std::chrono;
  milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  return ms.count();
}