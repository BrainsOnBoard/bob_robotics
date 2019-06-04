#pragma once

// BoB robotics includes
#include "common/semaphore.h"
#include "net/connection.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <atomic>

namespace BoBRobotics {
namespace Net {
class IMUNetSource {
public:
    IMUNetSource(Net::Connection &);
    units::angle::radian_t getYaw();

private:
    Net::Connection &m_Connection;
    Semaphore m_Semaphore;
    std::atomic<double> m_Yaw; // in radians

}; // IMUNetSource
} // Net
} // BoBRobotics
