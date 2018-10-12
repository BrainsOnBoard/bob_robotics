#pragma once

// Standard C++ includes
#include <atomic>
#include <memory>
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
    //! Stop the background thread (if needed)
    virtual ~Threadable()
    {
        stop();
    }

    //! Run on the current thread, blocking until process ends
    virtual void run() = 0;

    //! Run the process on a background thread
    virtual void runInBackground()
    {
        m_Thread = std::make_unique<std::thread>([this] { run(); });
    }

    //! Stop the background thread
    virtual void stop()
    {
        if (m_DoRun.exchange(false) && m_Thread) {
            if (m_Thread->joinable()) {
                m_Thread->join();
            } else {
                m_Thread->detach();
            }
        }
    }

private:
    std::unique_ptr<std::thread> m_Thread;

protected:
    std::atomic<bool> m_DoRun{ true };
};
}
