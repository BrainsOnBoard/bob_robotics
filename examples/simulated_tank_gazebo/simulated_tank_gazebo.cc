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
// Gazebo includes
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

//OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
void OnImageMsg(ConstImageStampedPtr &msg)
{
    // std::cout << msg->image().width() << std::endl;
    // std::cout << msg->image().height() << std::endl;
    // std::cout << msg->image().pixel_format() << std::endl;
    // std::cout << std::endl;

    int width;
    int height;
    char *data;

    width = (int) msg->image().width();
    height = (int) msg->image().height();
    data = new char[msg->image().data().length() + 1];

    memcpy(data, msg->image().data().c_str(), msg->image().data().length());
    cv::Mat image(height, width, CV_8UC3, data);

    cv::imshow("camera", image);
    cv::waitKey(1);
    delete data;  // DO NOT FORGET TO DELETE THIS, 
                  // ELSE GAZEBO WILL TAKE ALL YOUR MEMORY
}
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
    // Create our node for publishing joystick values
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    //Setup image subscription node
    gazebo::transport::NodePtr imageNode(new gazebo::transport::Node());
    gazebo::transport::SubscriberPtr imageSub;
    imageNode->Init();
    // Subscribe to the topic, and register a callback
    imageSub = imageNode->Subscribe("/gazebo/default/my_simple_cart/camera/link/camera/image", OnImageMsg);
    std::cerr << "Subsribed to /gazebo/default/my_simple_cart/camera/link/camera/image \n";

    
    // Publish to the  simple_cart topic
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
        LOG_DEBUG << wheelSpeeds.first << "," << wheelSpeeds.second;       

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
