#pragma once

// Gazebo includes
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>

namespace BoBRobotics {
namespace Gazebo {
auto
getNode()
{
    // Load gazebo as a client
    gazebo::client::setup(0, 0);
    
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
} // Gazebo
} // BoBRobotics
