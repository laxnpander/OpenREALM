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

#ifndef OPENREALM_TIMER_H
#define OPENREALM_TIMER_H

#include <thread>
#include <chrono>
#include <functional>
#include <atomic>

namespace realm {

    class Timer
    {
    public:
        using Ptr = std::shared_ptr<Timer>;
        using ConstPtr = std::shared_ptr<const Timer>;

    public:
        explicit Timer(const std::chrono::milliseconds &period, const std::function<void()> &func);
        ~Timer();

    private:
        std::chrono::milliseconds _period;
        std::function<void()> _func;
        std::atomic<bool> _in_flight;
        std::thread _thread;
    };

} // namespace realm

#endif //OPENREALM_TIMER_H
