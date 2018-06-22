#include "os/video.h"

int main()
{
    std::cout << "Cameras:" << std::endl;
    for (auto cam : BoBRobotics::OS::Video::getCameras()) {
        std::cout << "- Device " << cam.first << ": " << cam.second << std::endl;
    }
}
