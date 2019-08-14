// BoB robotics includes
#include "common/semaphore.h"

namespace BoBRobotics {
//! Stop wait() blocking on another thread if it's running
void
Semaphore::notify()
{
    std::unique_lock<std::mutex> lock(mtx);
    fired = true;
    cv.notify_one();
}

//! Wait for notify() to be invoked by another thread
void
Semaphore::wait()
{
    std::unique_lock<std::mutex> lock(mtx);
    while (!fired) {
        cv.wait(lock);
    }
    fired = false;
}

//! Wait for notify() to be invoked by another thread, but do not reset
void
Semaphore::waitOnce()
{
    std::unique_lock<std::mutex> lock(mtx);
    while (!fired) {
        cv.wait(lock);
    }
}
} // BoBRobotics
