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

#define LOGURU_IMPLEMENTATION 1
#include <realm_core/loguru.h>

#include <functional>

#include <realm_core/worker_thread_base.h>

using namespace realm;

WorkerThreadBase::WorkerThreadBase(const std::string &thread_name, bool verbose)
: _thread_name(thread_name),
  _finish_requested(false),
  _reset_requested(false),
  _stop_requested(false),
  _is_stopped(false),
  _verbose(verbose)
{
}

void WorkerThreadBase::start()
{
  startCallback();
  _thread = std::thread(std::bind(&WorkerThreadBase::run, this));
}

void WorkerThreadBase::join()
{
  _thread.join();
}

void WorkerThreadBase::run()
{
  LOG_IF_F(INFO, _verbose, "Thread '%s' starting loop...", _thread_name.c_str());
  while (!isFinishRequested())
  {
    // Calls to derived classes implementation of process()
    long t = getCurrentTimeMilliseconds();
    if (process())
    {
      LOG_IF_F(INFO,
               _verbose,
               "Thread '%s' has processed data. Time elapsed: %4.2f [s]",
               _thread_name.c_str(),
               static_cast<double>(getCurrentTimeMilliseconds() - t) / 1000);
    }
    // Handle stops and finish
    if (isStopRequested())
    {
      LOG_IF_F(INFO, _verbose, "Thread '%s' stopped!", _thread_name.c_str());
      while (isStopped() && !isFinishRequested())
      {
        if (isResetRequested())
        {
          reset();
          LOG_IF_F(INFO, _verbose, "Thread '%s' reseted!", _thread_name.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      LOG_IF_F(INFO, _verbose, "Thread '%s' resumed to loop!", _thread_name.c_str());
    }
    // Check if reset was requested and execute if neccessary
    if (isResetRequested())
    {
      reset();
      LOG_IF_F(INFO, _verbose, "Thread '%s' reseted!", _thread_name.c_str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  LOG_IF_F(INFO, _verbose, "Thread '%s' finished!", _thread_name.c_str());
}

void WorkerThreadBase::resume()
{
  std::unique_lock<std::mutex> lock(_mutex_is_stopped);
  if (_is_stopped)
    _is_stopped = false;
}

void WorkerThreadBase::requestStop()
{
  std::unique_lock<std::mutex> lock(_mutex_stop_requested);
  _stop_requested = true;
  LOG_IF_F(INFO, _verbose, "Thread '%s' received stop request...", _thread_name.c_str());
}

void WorkerThreadBase::requestReset()
{
  std::unique_lock<std::mutex> lock(_mutex_reset_requested);
  _reset_requested = true;
  LOG_IF_F(INFO, _verbose, "Thread '%s' received reset request...", _thread_name.c_str());
}

void WorkerThreadBase::requestFinish()
{
  std::unique_lock<std::mutex> lock(_mutex_finish_requested);
  _finish_requested = true;
  LOG_IF_F(INFO, _verbose, "Thread '%s' received finish request...", _thread_name.c_str());
  finishCallback();
}

bool WorkerThreadBase::isStopRequested()
{
  std::unique_lock<std::mutex> lock(_mutex_stop_requested);
  std::unique_lock<std::mutex> lock1(_mutex_is_stopped);
  if (_stop_requested && !_is_stopped)
  {
    _is_stopped = true;
  }
  return _stop_requested;
}

bool WorkerThreadBase::isResetRequested()
{
  std::unique_lock<std::mutex> lock(_mutex_reset_requested);
  return _reset_requested;
}

bool WorkerThreadBase::isFinishRequested()
{
  std::unique_lock<std::mutex> lock(_mutex_finish_requested);
  return _finish_requested;
}

bool WorkerThreadBase::isStopped()
{
  std::unique_lock<std::mutex> lock(_mutex_is_stopped);
  if (_is_stopped)
    _stop_requested = false;
  return _is_stopped;
}

long WorkerThreadBase::getCurrentTimeMilliseconds()
{
  using namespace std::chrono;
  milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  return ms.count();
}