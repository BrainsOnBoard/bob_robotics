// BoB robotics includes
#include "common/logging.h"
#include "os/video.h"

// Standard C++ includes

int main()
{
    LOGI << "Cameras:";
    for (auto cam : BoBRobotics::OS::Video::getCameras()) {
        LOGI << "- Device " << cam.first << ": " << cam.second;
    }
}
