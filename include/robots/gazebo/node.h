#pragma once

// Gazebo includes
#include <gazebo/transport/transport.hh>

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {
gazebo::transport::NodePtr
getNode();

void
shutDown();
} // Robots
} // Gazebo
} // BoBRobotics
