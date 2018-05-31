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
class Motor
{
public:
    virtual ~Motor()
    {}

    //----------------------------------------------------------------------------
    // Declared virtuals
    //----------------------------------------------------------------------------
    virtual void tank(float left, float right)
    {
        std::cout << "Dummy motor: left: " << left << "; right: " << right
                  << std::endl;
    }

    void readFromNetwork(Net::Node &node)
    {
        // handle incoming TNK commands
        node.addCommandHandler("TNK", [this] (Net::Node &node, const Net::Command &command) {
            onCommandReceived(node, command);
        });
    }

private:
    void onCommandReceived(Net::Node &node, const Net::Command &command)
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
}; // Motor
} // Robots
} // GeNNRobotics
