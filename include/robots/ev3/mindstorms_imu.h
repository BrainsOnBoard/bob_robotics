#pragma once

// Third-party includes
#include "third_party/units.h"

// EV3 library
#include "../../third_party/ev3dev-lang-cpp/ev3dev.h"

// Standard C++ includes
#include <utility>

namespace BoBRobotics {
class MindstormsIMU {
    using degree_t = units::angle::degree_t;
    using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;

public:
    MindstormsIMU(const ev3dev::address_type &address = ev3dev::INPUT_AUTO);
    degree_t getYaw();
    degrees_per_second_t getYawVelocity();
    std::pair<degree_t, degrees_per_second_t> getYawAndVelocity();

private:
    ev3dev::gyro_sensor m_IMU;
};
} // BoBRobotics
