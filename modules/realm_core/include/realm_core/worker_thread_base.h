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

#ifndef PROJECT_WORKER_THREAD_H
#define PROJECT_WORKER_THREAD_H

#include <thread>
#include <chrono>
#include <mutex>
#include <string>

namespace realm
{

class WorkerThreadBase
{
  public:
    /*!
     * @brief Basic constructor for worker thread class
     * @param thread_name Name of the thread
     * @param sleep_time Duration the thread is sleeping after processing in milliseconds
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

  protected:

    /*!
     * @brief Threader member for worker
     */
    std::thread _thread;

    /*!
     * @brief Time the processing thread sleeps in milliseconds
     */
    int64_t _sleep_time;

    /*!
     * @brief Verbose flag, set true if additional output should be generated
     */
    bool _verbose;

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
    virtual void finishCallback() {};

    /*!
     * @brief Name of the thread. Will be used to output current state
     */
    std::string _thread_name;

    /*!
     * @brief Thread management: set true, if a stop was requested from the outside. Will afterwards be set false
     * again, if "isStopped()" is triggered with "true".
     */
    bool _stop_requested;
    std::mutex _mutex_stop_requested;

    /*!
     * @brief Thread management: set true, if a reset was requested from the outside. Will afterwards be set false
     * again, if reset was successfully be executed.
     */
    bool _reset_requested;
    std::mutex _mutex_reset_requested;

    /*!
     * @brief Thread management: set true, if a stop was requested from the outside. Will never be set false again,
     * because stage will be closed as soon as possible.
     */
    bool _finish_requested;
    std::mutex _mutex_finish_requested;

    /*!
     * @brief Thread management: set true, if a stop was requested and has reached the stopping point. Will be set
     * false again, if resume was triggered
     */
    bool _is_stopped;
    std::mutex _mutex_is_stopped;

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
