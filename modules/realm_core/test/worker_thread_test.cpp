

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

TEST(WorkerThread, Async)
{
  // This test is about the asynchronous functionalities of the worker thread. Normally, it waits until the sleeping time
  // is over to return to processing. However, the user can provide a condition which is evaluated. If it returns true,
  // then the worker thread wakes up and processes as long as the condition is false again.
  auto worker = std::make_shared<DummyWorker>();
  worker->setSleepTime(1000);
  worker->registerAsyncConditionFunctor([=]{ return (worker->counter < 5 && worker->counter > 1); });
  worker->start();

  // So what's going to happen now is, that counter is immediately incremented in the worker. After that it falls asleep
  // for 1s. However, we are going to increment the counter from the main thread and notify the thread about the change.
  // The counter will then be 2 and therefore the defined condition above holds true until the counter is equal to 5.
  // After that it should fall asleep again and only wake up every second.

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(worker->counter, 1);

  {
    // Worker is sleeping, but we don't let him. We change the counter to meet the async condition
    worker->counter = 2;
    worker->notify();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Now 200ms are passed, but due to our async wakeup the counter should be at 5 already.
    EXPECT_EQ(worker->counter, 5);
  }

  {
    // We double check our notify by doing this again, setting the counter to 2. But this time we don't notify the thread,
    // So it should still be sleeping and not increment.
    worker->counter = 2;
    // no notify

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    EXPECT_EQ(worker->counter, 2);
  }

  worker->requestFinish();
  worker->notify();
  worker->join();
}

TEST(WorkerThread, InvalidConstruction)
{
  EXPECT_ANY_THROW(auto worker = std::make_shared<DummyWorker>(0););
}