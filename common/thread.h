#pragma once

// BoB robotics includes
#include "background_exception_catcher.h"

// Standard C++ includes
#include <atomic>
#include <memory>
#include <stdexcept>
#include <thread>

namespace BoBRobotics {
//! A simple RAII wrapper for std::thread
template<bool JoinOnDestroy=true>
class Thread {
public:
    template<class... Args>
    Thread(Args&&... args)
      : m_Thread(std::forward<Args>(args)...)
    {}

    Thread(Thread<JoinOnDestroy> &&thread)
    {
        m_Thread = std::move(thread.m_Thread);
    }

    ~Thread()
    {
        if (m_Thread.joinable()) {
            if (JoinOnDestroy) {
                m_Thread.join();
            } else {
                m_Thread.detach();
            }
        }
    }

    void operator=(Thread<JoinOnDestroy> &&thread)
    {
        m_Thread = std::move(thread.m_Thread);
    }

private:
    std::thread m_Thread;
}; // Thread

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
    Threadable()
      : m_DoRun(false)
    {}

    virtual ~Threadable()
    {}

    //! Run on the current thread, blocking until process ends
    virtual void run()
    {
        m_DoRun = true;
        runInternal();
    }

    //! Check if the run() function has been called
    virtual bool isRunning()
    {
        return m_DoRun;
    }

    //! Run the process on a background thread
    virtual void runInBackground()
    {
        m_Thread = Thread<>(&Threadable::runCatchExceptions, this);
    }

    //! Stop the background thread
    virtual void stop()
    {
        m_DoRun = false;
    }

    Threadable(const Threadable &old) = delete;
    void operator=(const Threadable &old) = delete;
    Threadable(Threadable &&old)
      : m_Thread(std::move(old.m_Thread))
      , m_DoRun(old.m_DoRun.load())
    {}
    Threadable &operator=(Threadable &&old) = default;

private:
    Thread<> m_Thread;
    std::atomic<bool> m_DoRun;

    void runCatchExceptions()
    {
        try {
            run();
        } catch (...) {
            BackgroundExceptionCatcher::set(std::current_exception());
        }
    }

protected:
    virtual void runInternal() = 0;
}; // Threadable
} // BoBRobotics
