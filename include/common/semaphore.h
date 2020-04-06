#pragma once

// Standard C++ includes
#include <condition_variable>
#include <mutex>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::Semaphore
//----------------------------------------------------------------------------
/*!
 * \brief A basic C++ semaphore class
 *
 * A C++11 semaphore implementation (after:
 * https://stackoverflow.com/questions/4792449/c0x-has-no-semaphores-how-to-synchronize-threads).
 */
class Semaphore
{
public:
    //! Stop wait() blocking on another thread if it's running
    void notify();

    //! Wait for notify() to be invoked by another thread
    void wait();

    //! Wait for notify() to be invoked by another thread, but do not reset
    void waitOnce();

private:
    std::condition_variable cv;
    std::mutex mtx;
    bool fired = false;
};
} // BoBRobotics
