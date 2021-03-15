

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

        static long getCurrentTimeSeconds();
        static long getCurrentTimeMilliseconds();
        static long getCurrentTimeMicroseconds();
        static long getCurrentTimeNanoseconds();

    private:
        std::chrono::milliseconds m_period;
        std::function<void()> m_func;
        std::atomic<bool> m_in_flight;
        std::thread m_thread;
    };

} // namespace realm

#endif //OPENREALM_TIMER_H
