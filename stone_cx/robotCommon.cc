#include "robotCommon.h"

// Standard C++ includes
#include <iostream>
#include <numeric>

// Common includes
#include "../common/motor_i2c.h"

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

float driveMotorFromCPU1(MotorI2C &motor, bool display)
{
    // Sum left and right motor activity
    const float leftMotor = std::accumulate(&rCPU1[0], &rCPU1[8], 0.0f);
    const float rightMotor = std::accumulate(&rCPU1[8], &rCPU1[16], 0.0f);

    // Steer based on signal
    const float steering = leftMotor - rightMotor;
    if(display) {
        std::cout << "Steer:" << steering << std::endl;
    }
    motor.tank(1.0f + (4.0f * steering), 1.0f - (4.0f * steering));
    return steering;
}
