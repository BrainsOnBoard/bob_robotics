#pragma once

namespace BoBRobotics {
namespace Robots {
// Forward declarations
class Tank;
class TANK_TYPE;
}

namespace StoneCX {
// Functions
float driveMotorFromCPU1(BoBRobotics::Robots::Tank &motor, bool display = false);
}
}