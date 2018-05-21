#pragma once

// C++ includes
#include <iostream>

// GeNN robotics includes
#include "motor.h"

namespace GeNN_Robotics {
namespace Robots {
class MotorDummy : public Motor
{
public:
    virtual void tank(float left, float right)
    {
        std::cout << "Dummy motor: left: " << left << "; right: " << right
                  << std::endl;
    }
}; // MotorDummy
} // Robots
} // GeNN_Robotics
