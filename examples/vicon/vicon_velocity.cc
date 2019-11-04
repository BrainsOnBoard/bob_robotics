// BoB robotics includes
#include "common/logging.h"
#include "vicon/capture_control.h"
#include "vicon/udp.h"

// Standard C++ includes
#include <chrono>
#include <cstdlib>
#include <thread>

using namespace BoBRobotics::Vicon;
using namespace std::literals;
using namespace units::angular_velocity;

int main()
{
    UDPClient<ObjectDataVelocity> vicon(51001);
    CaptureControl viconCaptureControl("192.168.1.100", 3003, "c:\\users\\ad374\\Desktop");

    viconCaptureControl.startRecording("test1");

    for (int i = 0; i < 120; i++) {
        auto objectData = vicon.getObjectData();
        const auto velocity = objectData.getVelocity();
        const auto angularVelocity = objectData.getAngularVelocity();

        LOGI << "("
             << velocity[0] << ", " << velocity[1] << ", " << velocity[2]
             << ") | ("
             << angularVelocity[0] << ", " << angularVelocity[1] << ", " << angularVelocity[2]
             << ")";

        std::this_thread::sleep_for(1s);
    }

    viconCaptureControl.stopRecording("test1");

    return EXIT_SUCCESS;
}
