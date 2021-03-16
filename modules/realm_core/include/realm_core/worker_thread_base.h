

#ifndef PROJECT_WORKER_THREAD_H
#define PROJECT_WORKER_THREAD_H

#include <thread>
#include <chrono>
#include <mutex>
#include <string>
#include <condition_variable>
#include <functional>

namespace realm
{

/**
 * A simple structure to hold statistics about the worker thread or derived
 * stage_base;
 */
struct Statistics
{
  double max;
  double min;
  double avg;
  long count;
};

class WorkerThreadBase
{
  public:

    /*!
     * @brief Basic constructor for worker thread class
     * @param thread_name Name of the thread
     * @param sleep_time Duration the thread is sleeping after processing in milliseconds. Note that we implemented
     * it with a condition variable, so you can also asynchronously wake up the processing thread by calling the notify()
     * function.
     * @param verbose Flag for verbose output
     */
    explicit WorkerThreadBase(const std::string &thread_name, int64_t sleep_time, bool verbose);

    /*!
     * @brief Starting function for thread
     */
    void start();

    /*!
     * @brief Joining the thread of the worker from outside the class
     */
    void join();

    /*!
     * @brief Loop of the stage thread. Handles calls to stop, reset, finish and process() for the derived stages.
     */
    void run();

    /*!
     * @brief Stopped main loop can be resumed through this function
     */
    void resume();

    /*!
     * @brief Other threads, most likely the communication thread, can request the processing thread to be stopped
     * for whatever reason there might be.
     */
    void requestStop();

    /*!
     * @brief Requesting a reset can have several reasons. The most trivial is that a mission is completed and another
     * one is about to start. Processing thread will execute as soon as the current image is done.
     */
    void requestReset();

    /*!
     * @brief Requesting a finish will end the thread and call the destructor afterwards.
     */
    void requestFinish();

    /*!
     * @brief Allows the processing thread to return before sleep time is over.
     */
    void notify();

  protected:

    /*!
     * @brief Threader member for worker
     */
    std::thread m_thread;

    /*!
     * @brief Time the processing thread sleeps in milliseconds
     */
    int64_t m_sleep_time;

    /*!
     * @brief Verbose flag, set true if additional output should be generated
     */
    bool m_verbose;

    /*!
     * @brief Stores max/min/avg time to process a task
     */
    Statistics m_process_statistics{};
    std::mutex m_mutex_statistics{};

    std::mutex m_mutex_processing;
    std::function<bool()> m_data_ready_functor;
    std::condition_variable m_condition_processing;

    /*!
     * @brief Function that every derived worker thread should implement. run() will trigger process, if no stop, reset or
     * finish is requested from the main thread. process() should contain the general workflow.
     * @return Returns true, if processing was performed and not empty looped
     */
    virtual bool process() = 0;

    /*!
     * @brief Function that every derived worker thread may implement. After start was triggered, this function will be
     * called.
     */
    virtual void startCallback() {};

    /*!
     * @brief Function that every derived worker thread may implement. After finish was requested, this function will be
     * called. Typically last calls to save functions can be put in the implementation.
     */
    virtual void finishCallback() {}

    /*!
     * @return Returns the average, max, and min time the processing step is taking
     */
    Statistics getProcessingStatistics();

    /*!
     * @brief Name of the thread. Will be used to output current state
     */
    std::string m_thread_name;

    /*!
     * @brief Thread management: set true, if a stop was requested from the outside. Will afterwards be set false
     * again, if "isStopped()" is triggered with "true".
     */
    bool m_stop_requested;
    std::mutex m_mutex_stop_requested;

    /*!
     * @brief Thread management: set true, if a reset was requested from the outside. Will afterwards be set false
     * again, if reset was successfully be executed.
     */
    bool m_reset_requested;
    std::mutex m_mutex_reset_requested;

    /*!
     * @brief Thread management: set true, if a stop was requested from the outside. Will never be set false again,
     * because stage will be closed as soon as possible.
     */
    bool m_finish_requested;
    std::mutex m_mutex_finish_requested;

    /*!
     * @brief Thread management: set true, if a stop was requested and has reached the stopping point. Will be set
     * false again, if resume was triggered
     */
    bool m_is_stopped;
    std::mutex m_mutex_is_stopped;

    /*!
     * @brief virtual function for the derived stage to be implemented. Has to reset all neccessary data to allow a
     * fresh new start of the stage.
     */
    virtual void reset() = 0;

    /*!
     * @brief Calculates current time since unix epoch in seconds with milliseconds accuracy
     * @return Seconds since unix epoch with milliseconds accuracy
     */
    static long getCurrentTimeMilliseconds();

    /*!
     * @brief Simple getter for the derived stage to see if client has requested a stop.
     * @return true if stop was requested, false if not.
     */
    bool isStopRequested();

    /*!
     * @brief Simple getter for the derived stage to see if client has requested a finish. Typically for the main run()
     * loop to check if running is still required, e.g. while(!isFinishRequested()) { do_magic; }
     * @return true if finish was requested, false if not.
     */
    bool isFinishRequested();

    /*!
     * @brief Simple getter for the derived stage to see if client has requested a reset.
     * @return true if reset was requested, false if not.
     */
    bool isResetRequested();

    /*!
     * @brief Simple threadsafe getter for the derived stage to see if resume() has not been called yet.
     * @return true if resume is currently not wanted, false if resume required.
     */
    bool isStopped();

};

} // namespace realm

#endif //PROJECT_WORKER_THREAD_H
