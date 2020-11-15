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

#ifndef OPENREALM_TEST_HELPER_H
#define OPENREALM_TEST_HELPER_H

#include <memory>

#include <realm_core/camera.h>
#include <realm_core/frame.h>
#include <realm_core/settings_base.h>
#include <realm_core/worker_thread_base.h>

namespace realm {

  cv::Mat createDummyPose();
  camera::Pinhole createDummyPinhole();
  Frame::Ptr createDummyFrame();

  class DummySettings : public SettingsBase
  {
  public:
    DummySettings()
    {
      add("parameter_string", Parameter_t<std::string>{"dummy_string", "This is a dummy string."});
      add("parameter_double", Parameter_t<double>{2.4, "This is a dummy double."});
      add("parameter_int", Parameter_t<int>{5, "This is a dummy integer."});
    }
  };

  class DummyWorker : public WorkerThreadBase
  {
  public:
    explicit DummyWorker(int64_t sleep_time = 100) : WorkerThreadBase("dummy_worker", sleep_time, false), counter(0) {}
    bool process() override { counter++; };
    void reset() override { counter = 0; };

    void registerAsyncConditionFunctor(const std::function<bool()> &func) { _data_ready_functor = ([=]{ return func() || isFinishRequested();}); };
    void setSleepTime(int64_t sleep_time) { _sleep_time = sleep_time; };

    volatile int counter;
  };

} // namespace realm


#endif //OPENREALM_TEST_HELPER_H
