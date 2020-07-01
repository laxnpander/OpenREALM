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

#include <iostream>

#include "test_helper.h"

// gtest
#include <gtest/gtest.h>

using namespace realm;

TEST(WorkerThread, Basics)
{
  // Here we test basic functions of the worker thread class. Because it is a purely abstract class we have to use a
  // derived child to test it. Dummy worker increments an integer every time it processes. Therefore we run it for a while
  // and check if the incremented value is as expected.
  auto worker = std::make_shared<DummyWorker>();
  worker->start();

  // The dummy thread is processing rather slow (every 100 ms), so we don't create any race condition.
  std::this_thread::sleep_for(std::chrono::milliseconds(450));
  EXPECT_EQ(worker->counter, 5);

  // Stop requests are checked before processing, therefore the counter could not have changed
  worker->requestStop();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(worker->counter, 5);

  // Resets should also be processed when thread is stopped
  worker->requestReset();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(worker->counter, 0);

  // Depending on where in the loop the thread currently is, the counter will have different values. But it should be greater 0.
  worker->resume();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  EXPECT_TRUE(worker->counter > 0);

  worker->requestFinish();
  worker->join();
}