#pragma once

// C++ includes
#include <string>

// GeNN robotics includes
#include "../net/node.h"

namespace GeNNRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// Motor
//----------------------------------------------------------------------------
// Interface for driving tank-like wheeled robots
class Motor : public Net::Handler
{
public:
    virtual ~Motor()
    {}

    //----------------------------------------------------------------------------
    // Declared virtuals
    //----------------------------------------------------------------------------
    virtual void tank(float left, float right) = 0;

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
        tank(left, right);
    }

    std::string getHandledCommandName() const override
    {
        return "TNK";
    }
}; // Motor
} // Robots
} // GeNNRobotics
