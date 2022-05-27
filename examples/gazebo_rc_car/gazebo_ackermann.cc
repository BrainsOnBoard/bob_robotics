// BoB robotics includes
#include "robots/gazebo/rc_car.h"
#include "robots/gazebo/camera.h"
#include "robots/gazebo/node.h"
#include "video/display.h"

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics::Robots;
using namespace std::literals;
using namespace units::literals;


int main(int argc, char **argv)
{
    /************************************Gazebo setup************/

    // Create our node for publishing keyboard values
    gazebo::transport::NodePtr node = Gazebo::getNode();

    /************************************Gazebo setup end************/
    std::unique_ptr<Video::Display> display;
    std::unique_ptr<Gazebo::Camera> cam;
    if(argc >= 3) { // Initialize gazebo camera if more than 2 arguements are provided (display switch and camera url)
        std::cout << "Display switch enabled.\n";
        if(strcmp(argv[1], "-p") == 0) {
            std::cout << "Using panoramic camera.\n";
            cam = std::make_unique<Gazebo::Camera>(node, argv[2], true);
            display = std::make_unique<Video::Display>(*cam, cv::Size(640,320)); //unwrap resolution needs to be supplied
        }
        else if(strcmp(argv[1], "-s") == 0){
            std::cout << "Using simple camera.\n";
            cam = std::make_unique<Gazebo::Camera>(node, argv[2], false);
            display = std::make_unique<Video::Display>(*cam); //unwrap resolution needs to be supplied
        }

        //display->runInBackground();
    }


    BoBRobotics::Robots::Gazebo::RCCar robot(5_rad_per_s, node); // car agent
    robot.start(node);

    for(;;) {
        // Check for keyboard events
        if (1) { // if keyboard event
            cv::Mat frame;
            display->update();


            //cv::imshow("frame" ,frame);
            //cv::waitKey(1);
            // A small delay so we don't hog CPU
            //robot.move(1,0);
            std::this_thread::sleep_for(5ms);
        }
    }

    // Make sure to shut everything down.
    display->close();
    Gazebo::shutDown();
    std::cout <<"Shutting down...\n";
}
