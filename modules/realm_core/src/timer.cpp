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