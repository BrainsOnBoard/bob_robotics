#pragma once

// Standard C++ includes
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
    virtual ~Threadable()
    {
        stop();
    }

    virtual void run() = 0;

    virtual void runInBackground()
    {
        m_Thread = std::thread([this] { run(); });
        m_ThreadRunning = true;
    }

    virtual void stop()
    {
        m_DoRun = false;
        if (m_ThreadRunning) {
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
    bool m_DoRun = true;
};
}
