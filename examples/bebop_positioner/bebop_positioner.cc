// BoB robotics includes
#include "common/plot_agent.h"
#include "hid/joystick.h"
#include "libbebop/bebop.h"
#include "vicon/udp.h"

// Standard C++ includes
#include <chrono>
#include <exception>
#include <iostream>

using namespace std::literals;
using namespace BoBRobotics;
namespace plt = matplotlibcpp;

int main()
{
    try {
        // Xbox controller
        HID::Joystick joystick;

        // UAV object
        Robots::Bebop uav;
        uav.addJoystick(joystick);

        // Motion-tracking system
        Vicon::UDPClient<> vicon(51001);
        vicon.waitForObject();

        do {
            // Check for joystick events
            joystick.update();

            // Plot UAV's position
            plt::figure(1);
            plotAgent(vicon.getObjectData(0), { -2500, 2500 }, { -2500, 2500 });
            plt::pause(0.025);
        } while (plt::fignum_exists(1));
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
    }
}