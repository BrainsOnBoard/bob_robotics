/*
 * A simple abstract class representing a common interface for running processes
 * either in the foreground or background (e.g. displaying camera, handling
 * joystick etc.).
 */

#pragma once

// C++ includes
#include <memory>
#include <thread>

namespace GeNNRobotics {
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
    }

    virtual void stop()
    {
        m_DoRun = false;
        waitToFinish();
    }

    virtual void waitToFinish()
    {
        if (m_Thread.joinable()) {
            m_Thread.join();
        } else {
            m_Thread.detach();
        }
    }

private:
    std::thread m_Thread;

protected:
    bool m_DoRun = true;
};
}
