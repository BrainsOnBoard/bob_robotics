#pragma once

// Standard C++ includes
#include <atomic>
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
    Thread<> m_Thread;
    std::atomic<bool> m_DoRun;

    void runCatchExceptions();

protected:
    virtual void runInternal() = 0;
}; // Threadable
} // BoBRobotics
