#pragma once

namespace GeNNRobotics {
namespace Robots {
// Forward declarations
class MotorI2C;
}
}

// Functions
float driveMotorFromCPU1(GeNNRobotics::Robots::MotorI2C &motor, bool display = false);
