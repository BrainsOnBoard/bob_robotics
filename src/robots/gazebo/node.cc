// BoB robotics includes
#include "robots/gazebo/node.h"

// Gazebo
#include "gazebo/gazebo_client.hh"

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {
gazebo::transport::NodePtr
getNode()
{
    // Load gazebo as a client
    gazebo::client::setup(0, nullptr);

    // Create our node for publishing joystick values
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    return node;
}

void
shutDown()
{
    gazebo::client::shutdown();
}
}
}
}
