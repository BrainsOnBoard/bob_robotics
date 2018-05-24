#pragma once

// C++ includes
#include <string>

// GeNN robotics includes
#include "../net/node.h"

// local includes
#include "motor.h"

namespace GeNNRobotics {
namespace Robots {
class MotorNetSource
{
public:
    MotorNetSource(Net::Node *node, Motor *motor)
      : m_Motor(motor)
    {
        node->addHandler("TNK", handleCommand, this);
    }

private:
    Motor *m_Motor;

    void handleCommand(Net::Command &command)
    {
        // second space separates left and right parameters
        if (command.size() != 3) {
            throw Net::bad_command_error();
        }

        // parse strings to floats
        const float left = stof(command[1]);
        const float right = stof(command[2]);

        // send motor command
        m_Motor->tank(left, right);
    }

    static void handleCommand(Net::Command &command, void *userData)
    {
        reinterpret_cast<MotorNetSource *>(userData)->handleCommand(command);
    }
}; // MotorNetSource
} // Robots
} // GeNNRobotics
