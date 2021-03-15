

#include <realm_core/timer.h>

using namespace realm;

Timer::Timer(const std::chrono::milliseconds &period, const std::function<void()> &func)
    : m_period(period),
      m_func(func),
      m_in_flight(true)
{
    // Lambda implementation for threading
    m_thread = std::thread([this]
            {
                while (m_in_flight)
                {
                    std::this_thread::sleep_for(m_period);
                    if (m_in_flight)
                    {
                        m_func();
                    }
                }
            });
}

Timer::~Timer()
{
  m_in_flight = false;
    m_thread.join();
}

long Timer::getCurrentTimeSeconds()
{
  using namespace std::chrono;
  seconds s = duration_cast<seconds>(system_clock::now().time_since_epoch());
  return s.count();
}

long Timer::getCurrentTimeMilliseconds()
{
  using namespace std::chrono;
  milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  return ms.count();
}

long Timer::getCurrentTimeMicroseconds()
{
  using namespace std::chrono;
  microseconds ms = duration_cast<microseconds>(system_clock::now().time_since_epoch());
  return ms.count();
}

long Timer::getCurrentTimeNanoseconds()
{
  using namespace std::chrono;
  nanoseconds ns = duration_cast<nanoseconds>(system_clock::now().time_since_epoch());
  return ns.count();
}