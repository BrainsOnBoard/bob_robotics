#pragma once

// C++ includes
#include <string>

// GeNN robotics includes
#include "../net/node.h"

// local includes
#include "motor.h"

namespace GeNNRobotics {
namespace Robots {
class MotorNetSource : public Net::Handler
{
public:
    MotorNetSource(Motor *motor)
      : m_Motor(motor)
    {
    }

    void onCommandReceived(Net::Node &node, Net::Command &command) override
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

    std::string getHandledCommandName() const override
    {
        return "TNK";
    }

private:
    Motor *m_Motor;
}; // MotorNetSource
} // Robots
} // GeNNRobotics
