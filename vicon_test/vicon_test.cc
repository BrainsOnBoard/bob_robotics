// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// BoB robotics includes
#include "../vicon/capture_control.h"
#include "../vicon/udp.h"

using namespace BoBRobotics::Vicon;

int main()
{
    UDPClient<ObjectData> vicon(51001);
    CaptureControl viconCaptureControl("192.168.1.100", 3003,
                                              "c:\\users\\ad374\\Desktop");
    while(vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Waiting for object" << std::endl;
    }
    

    if(!viconCaptureControl.startRecording("test1")) {
        return EXIT_FAILURE;
    }
    for(int i = 0; i < 10000; i++) {
        auto objectData = vicon.getObjectData(0);
        const auto &translation = objectData.getTranslation();
        const auto &rotation = objectData.getRotation();

        std::cout << translation[0] << ", " << translation[1] << ", " << translation[2] << ", " << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << std::endl;
    }
    if(!viconCaptureControl.stopRecording("test1")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
