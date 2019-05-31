// BoB robotics includes
#include "os/video.h"

// Standard C++ includes
#include <iostream>

int main()
{
    std::cout << "Cameras:" << std::endl;
    for (auto cam : BoBRobotics::OS::Video::getCameras()) {
        std::cout << "- Device " << cam.first << ": " << cam.second << std::endl;
    }
}
