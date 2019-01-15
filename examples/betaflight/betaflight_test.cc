#include "betaflight_vicon.h"

// BoB robotics includes
#include "hid/joystick.h"
#include "robots/betaflight_uav.h"

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <chrono>
#include <cstdlib>
#include <future>
#include <iostream>
#include <thread>

using namespace std::literals;
using namespace BoBRobotics;

std::string
GetLineFromCin()
{
    std::string line;
    std::cin >> line;
    return line;
}

int
main(int argc, char *argv[])
{
    using namespace units::literals;
    using namespace units::angle;

    // can be set from the command line - default to LINUX standards
    const std::string device = (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    BoBRobotics::Robots::BetaflightVicon droneControl(device, baudrate);

    // limits for the VICON lab in Sheffield
    //droneControl.setRoomBounds(-2.2, 2.6, -4.2, 3.4 , 0.0, 2.0);
    droneControl.setRoomBounds(-1.5, 1.0, -1.0, 1.0, 0.0, 2.0);

    auto future = std::async(std::launch::async, GetLineFromCin);

    bool mute = true;
    bool controlOn = false;
    droneControl.printStatus();

    HID::Joystick js(0.1f);

    constexpr float speed = 0.05f;

    float yaw = 0.0f;
    Vector3<float> waypoint = { { 0.0f, 0.0f, 0.2f } };
    while (true) {
        js.update();

        // Move waypoint around
        waypoint[0] = std::max(droneControl.m_RoomBounds.x[0], std::min(droneControl.m_RoomBounds.x[1], waypoint[0] + (speed * js.getState(HID::JAxis::RightStickHorizontal))));
        waypoint[1] = std::max(droneControl.m_RoomBounds.y[0], std::min(droneControl.m_RoomBounds.y[1], waypoint[1] - (speed * js.getState(HID::JAxis::RightStickVertical))));
        waypoint[2] = std::max(droneControl.m_RoomBounds.z[0], std::min(droneControl.m_RoomBounds.z[1], waypoint[2] - (speed * js.getState(HID::JAxis::LeftStickVertical))));
        yaw = std::max(-180.0f, std::min(180.0f, yaw + (1.0f * js.getState(HID::JAxis::LeftStickHorizontal))));

        droneControl.setWaypoint(waypoint[0], waypoint[1], waypoint[2]);
        droneControl.setYaw(degree_t(yaw));

        if (js.isPressed(HID::JButton::X)) {
            break;
        }

        if (js.isPressed(HID::JButton::A)) {
            std::cout << "Turning on control and arming" << std::endl;
            droneControl.armDrone();
        }

        if (js.isPressed(HID::JButton::B)) {
            std::cout << "Landing" << std::endl;
            waypoint = { { 0.0f, 0.0f, 0.0f } };
        }

        if (js.isPressed(HID::JButton::RB)) {
            controlOn = true;
        }

        if (js.isPressed(HID::JButton::LB)) {
            std::cout << "Turning off control and disarming" << std::endl;
            droneControl.disarmDrone();
            controlOn = false;
        }

        if (js.isPressed(HID::JButton::Y)) {
            mute = !mute;
        }

        if (!mute) {
            std::cout << "target:" << waypoint[0] << ", " << waypoint[1] << ", " << waypoint[2] << std::endl;
            std::cout << "yaw:" << yaw << std::endl;
            droneControl.printStatus();
        }

        droneControl.sendCommands(controlOn);
    }

    droneControl.disarmDrone();

    // shut down threads
    exit(0);
}
