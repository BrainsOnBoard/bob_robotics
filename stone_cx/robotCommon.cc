#include "robotCommon.h"

// Standard C++ includes
#include <iostream>
#include <numeric>

// Common includes
#include "../common/motor_i2c.h"

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

void driveMotorFromCPU1(MotorI2C &motor, float steerThreshold, bool display)
{
    // Sum left and right motor activity
    const scalar leftMotor = std::accumulate(&rCPU1[0], &rCPU1[8], 0.0f);
    const scalar rightMotor = std::accumulate(&rCPU1[8], &rCPU1[16], 0.0f);

    // Steer based on signal
    const scalar steering = leftMotor - rightMotor;
    if(display) {
        std::cout << "Steer:" << steering << std::endl;
    }
    if(steering > steerThreshold) {
        motor.tank(1.0f, -1.0f);
    }
    else if(steering < -steerThreshold) {
        motor.tank(-1.0f, 1.0f);
    }
    else {
        motor.tank(1.0f, 1.0f);
    }
}