#pragma once

namespace GeNNRobotics {
namespace Robotics {
// Forward declarations
class MotorI2C;
}
}

// Functions
float driveMotorFromCPU1(GeNNRobotics::Robotics::MotorI2C &motor, bool display = false);
