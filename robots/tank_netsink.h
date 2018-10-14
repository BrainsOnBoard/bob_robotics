#pragma once

// BoB robotics includes
#include "../net/node.h"

// local includes
#include "tank.h"

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::TankNetSink
//----------------------------------------------------------------------------
//! An interface for transmitting tank steering commands over the network
class TankNetSink : public Tank
{
public:
    TankNetSink(Net::Node &node)
      : m_Node(&node)
    {}

    /* Motor command: send TNK command over TCP */
    void tank(float left, float right) override
    {
        BOB_ASSERT(left >= -1.f && left <= 1.f);
        BOB_ASSERT(right >= -1.f && right <= 1.f);

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
