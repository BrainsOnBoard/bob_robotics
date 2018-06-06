#pragma once

// GeNN robotics includes
#include "../net/node.h"

// local includes
#include "tank.h"

namespace GeNNRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// GeNNRobotics::Robots::TankNetSink
//----------------------------------------------------------------------------
class TankNetSink : public Tank
{
public:
    TankNetSink(Net::Node &node)
      : m_Node(&node)
    {}

    /* Motor command: send TNK command over TCP */
    void tank(float left, float right) override
    {
        // don't send a command if it's the same as the last one
        if (left == m_OldLeft && right == m_OldRight) {
            return;
        }

        // send steering command
        m_Node->getSocket()->send("TNK " + std::to_string(left) + " " +
                                  std::to_string(right) + "\n");

        // store current left/right values to compare next time
        m_OldLeft = left;
        m_OldRight = right;
    }

private:
    Net::Node *m_Node;
    float m_OldLeft = 0, m_OldRight = 0;
};
}
}
