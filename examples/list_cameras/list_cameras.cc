// BoB robotics includes
#include "plog/Log.h"
#include "os/video.h"

// Standard C++ includes

int bobMain()
{
    LOGI << "Cameras:";
    for (auto cam : BoBRobotics::OS::Video::getCameras()) {
        LOGI << "- Device " << cam.first << ": " << cam.second;
    }
    return EXIT_SUCCESS;
}
