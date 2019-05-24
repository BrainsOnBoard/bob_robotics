// BoB robotics includes
#include "common/main.h"
#include "hid/joystick.h"
#include "robots/simulated_tank.h"
#include "robots/gazebo_tank.h"
#include "common/sfml_world.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;

int
bob_main(int, char **)
{
    /************************************Gazebo setup************/
    // Load gazebo as a client
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(0, 0);
    #else
    gazebo::client::setup(0, 0);
    #endif
    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the  velodyne topic
    gazebo::transport::PublisherPtr pub =node->Advertise<gazebo::msgs::Vector3d>("~/my_simple_cart/vel_cmd");

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Create a a vector3 message
    gazebo::msgs::Vector3d msg;
    /************************************Gazebo setup end************/

    
    Robots::GazeboTank<> robot(5_mps); // Tank agent
    HID::Joystick joystick(0.25f);
    robot.controlWithThumbsticks(joystick);


    std::cout << "Drive the car using the two thumbsticks: each stick is for one motor" << std::endl;

    do {
        // Refresh display
        // car.setPose(robot.getPose());
        auto wheelSpeeds = robot.getWheelSpeeds();
          // Set the velocity in the x-component
        #if GAZEBO_MAJOR_VERSION < 6
        gazebo::msgs::Set(&msg, gazebo::math::Vector3((float)wheelSpeeds.first, (float)wheelSpeeds.second, 0));
        #else
        gazebo::msgs::Set(&msg, ignition::math::Vector3d((float)wheelSpeeds.first, (float)wheelSpeeds.second, 0));
        #endif
        // Send the message
        pub->Publish(msg);
        // std::cout<< wheelSpeeds.first << "," << wheelSpeeds.second << std::endl;        

        // Check for joystick events
        if (!joystick.update()) {
            // A small delay so we don't hog CPU
            std::this_thread::sleep_for(5ms);
        }
    } while (!joystick.isPressed(HID::JButton::B));
    // Make sure to shut everything down.
    #if GAZEBO_MAJOR_VERSION < 6
        gazebo::shutdown();
    #else
        gazebo::client::shutdown();
    #endif
    return EXIT_SUCCESS;
}
