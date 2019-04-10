#pragma once

// Standard C++ includes
#include <atomic>
#include <thread>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::Threadable
//----------------------------------------------------------------------------
/*!
 * \brief An abstract class allowing methods to run on the current thread or a
 *        backround thread
 *
 * A simple abstract class representing a common interface for running processes
 * either in the foreground or background (e.g. displaying camera, handling
 * joystick etc.).
 */
class Threadable
{
public:
    Threadable();
    virtual ~Threadable();

    //! Run on the current thread, blocking until process ends
    virtual void run();

    //! Check if the run() function has been called
    virtual bool isRunning();

    //! Run the process on a background thread
    virtual void runInBackground();

    //! Stop the background thread
    virtual void stop();

    Threadable(const Threadable &old) = delete;
    void operator=(const Threadable &old) = delete;
    Threadable(Threadable &&old);
    Threadable &operator=(Threadable &&old) = default;

private:
    std::thread m_Thread;
    std::atomic<bool> m_DoRun;

    void runCatchExceptions();

protected:
    virtual void runInternal() = 0;
};
}
