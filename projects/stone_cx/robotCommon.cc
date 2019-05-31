#define NO_HEADER_DEFINITIONS

#include "robotCommon.h"

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <numeric>

// Common includes
#include "robots/tank.h"

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

float BoBRobotics::StoneCX::driveMotorFromCPU1(BoBRobotics::Robots::Tank &motor, bool display)
{
    // Sum left and right motor activity
    const float leftMotor = std::accumulate(&rCPU1[0], &rCPU1[8], 0.0f);
    const float rightMotor = std::accumulate(&rCPU1[8], &rCPU1[16], 0.0f);

    // Steer based on signal
    const float steering = leftMotor - rightMotor;
    if(display) {
        std::cout << "Steer:" << steering << std::endl;
    }

    // Clamp motor input values to be between -1 and 1
    const float left = 1.0f + (4.0f * steering);
    const float right = 1.0f - (4.0f * steering);
    motor.tank(std::max(-1.f, std::min(1.f, left)), std::max(-1.f, std::min(1.f, right)));
    return steering;
}
