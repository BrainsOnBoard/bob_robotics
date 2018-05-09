#pragma once

#include "motor.h"
#include <iostream>

/* this is the superclass for other Motor devices */
class MotorDummy : public Motor
{
public:
    virtual void tank(float left, float right)
    {
        std::cout << "Dummy motor: left: " << left << "; right: " << right
                  << std::endl;
    }
};
