#include "os/video.h"

int
main(int argc, char **argv)
{
    std::cout << "Cameras:" << std::endl;
    for (auto cam : GeNN_Robotics::OS::Video::getCameras()) {
        std::cout << "- Device " << cam.number << ": " << cam.name << std::endl;
    }
}
