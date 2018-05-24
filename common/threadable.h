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
        m_Thread = std::unique_ptr<std::thread>(new std::thread([=] { run(); }));
    }

    virtual void stop()
    {
        m_DoRun = false;
        waitToFinish();
    }

    virtual void waitToFinish()
    {
        if (m_Thread && m_Thread->joinable()) {
            m_Thread->join();
        }
    }

private:
    std::unique_ptr<std::thread> m_Thread;

protected:
    bool m_DoRun = true;
};
}
