

#ifndef OPENREALM_TIMER_H
#define OPENREALM_TIMER_H

#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace realm {

    class Timer
    {
    public:
        using Ptr = std::shared_ptr<Timer>;
        using ConstPtr = std::shared_ptr<const Timer>;

    public:
        explicit Timer(const std::chrono::milliseconds &period, const std::function<void()> &func);
        ~Timer();


        /**
         * Waitfor that allows the timer to be interrupted for quicker shutdowns
         * @param duration The sleep time between updates
         * @return True if we need to abort
         */
        template<class Duration>
        bool interruptable_wait_for(Duration duration) {
            std::unique_lock<std::mutex> l(m_stop_mtx);
            return !m_cond.wait_for(l, duration, [this]() { return !m_in_flight; });
        }

        static long getCurrentTimeSeconds();
        static long getCurrentTimeMilliseconds();
        static long getCurrentTimeMicroseconds();
        static long getCurrentTimeNanoseconds();

    private:
        std::chrono::milliseconds m_period;
        std::function<void()> m_func;
        std::atomic<bool> m_in_flight;
        std::thread m_thread;

        std::condition_variable m_cond;
        std::mutex m_stop_mtx;
    };

} // namespace realm

#endif //OPENREALM_TIMER_H
