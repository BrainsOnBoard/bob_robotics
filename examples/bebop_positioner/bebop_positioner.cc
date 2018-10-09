// BoB robotics includes
#include "common/plot_agent.h"
#include "hid/joystick.h"
#include "libbebop/bebop.h"
#include "robots/uav_positioner.h"
#include "vicon/udp.h"

// Standard C++ includes
#include <chrono>
#include <exception>
#include <iostream>

using namespace std::literals;
using namespace units::literals;
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

        // Positioner
        Robots::UAVPositioner positioner(uav, 0.1f);
        bool positionerMode = false;
        joystick.addHandler([&positionerMode](HID::JButton button, bool pressed) {
            if (button == HID::JButton::Y && pressed) {
                positionerMode = !positionerMode;
                return true;
            } else {
                return false;
            }
        });

        do {
            // Check for joystick events
            joystick.update();

            if (positionerMode) {
                positioner.update(vicon.getObjectData(0), 0_deg);
            }

            // Plot UAV's position
            plt::figure(1);
            plotAgent(vicon.getObjectData(0), { -2500, 2500 }, { -2500, 2500 });
            plt::pause(0.025);
        } while (plt::fignum_exists(1));
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
    }
}