#pragma once
// Gazebo includes
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>

#include <gazebo/gazebo_client.hh>

using namespace gazebo::transport;
NodePtr getGazeboNode(){
    // Load gazebo as a client
    gazebo::client::setup(0, 0);
    // Create our node for publishing joystick values
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    return node;
}

void shutdownGazeboNode(){
    gazebo::client::shutdown();
}