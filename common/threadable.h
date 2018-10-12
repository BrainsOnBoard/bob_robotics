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
        m_Thread = std::thread([this] { run(); });
        m_ThreadRunning = true;
    }

    //! Stop the background thread
    virtual void stop()
    {
        if (m_DoRun.exchange(false) && m_ThreadRunning) {
            if (m_Thread.joinable()) {
                m_Thread.join();
            } else {
                m_Thread.detach();
            }
            m_ThreadRunning = false;
        }
    }

private:
    std::thread m_Thread;
    bool m_ThreadRunning = false;

protected:
    std::atomic<bool> m_DoRun{ true };
};
}
