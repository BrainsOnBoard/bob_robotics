// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/threadable.h"

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
Threadable::Threadable()
  : m_DoRun(false)
{}

Threadable::~Threadable()
{}

void
Threadable::run()
{
    m_DoRun = true;
    runInternal();
}

bool
Threadable::isRunning()
{
    return m_DoRun;
}

void
Threadable::runInBackground()
{
    m_Thread = std::thread(&Threadable::runCatchExceptions, this);
}

void
Threadable::stop()
{
    m_DoRun = false;
    if (m_Thread.joinable()) {
        m_Thread.join();
    }
}

Threadable::Threadable(Threadable &&old)
  : m_Thread(std::move(old.m_Thread))
  , m_DoRun(old.m_DoRun.load())
{}

void
Threadable::runCatchExceptions()
{
    try {
        run();
    } catch (...) {
        BackgroundExceptionCatcher::set(std::current_exception());
    }
}
} // BoBRobotics
