

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

long Timer::getCurrentTimeMilliseconds()
{
  using namespace std::chrono;
  milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  return ms.count();
}