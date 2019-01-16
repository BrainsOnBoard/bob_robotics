// BoB robotics includes
#include "hid/joystick.h"
#include "robots/betaflight_uav.h"
#include "vicon/uav_control.h"

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace std::literals;
using namespace units::angle;
using namespace units::literals;
using namespace units::length;
using namespace BoBRobotics;

void
printStatus(Robots::BetaflightUAV &drone, Vicon::UDPClient<Vicon::ObjectDataVelocity> &vicon)
{
    const std::string armState = drone.getArmState();
    if (!armState.empty()) {
        std::cout << armState << std::endl;
    }
    std::cout << "[V:" << std::setw(4) << drone.getVoltage() << " ";

    if (vicon.getNumObjects() > 0) {
        const auto objectData = vicon.getObjectData(0);
        const auto position = objectData.getPosition<>();
        const auto attitude = objectData.getAttitude<degree_t>();

        for (int i = 0; i < 3; ++i) {
            std::cout << std::setw(5) << std::fixed << std::setprecision(2) << position[i] << ", ";
        }
        for (int i = 0; i < 3; ++i) {
            std::cout << std::setw(6) << std::fixed << std::setprecision(2) << attitude[i];
            if (i < 2)
                std::cout << ", ";
        }

    } else {
        std::cout << "NO VICON DATA";
    }
    std::cout << "]" << std::endl;
}

int
main(int argc, char *argv[])
{
    // Joystick for controlling drone
    HID::Joystick js(0.1f);

    constexpr float speed = 0.05f;

    // can be set from the command line - default to LINUX standards
    const std::string device = (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    // Interface to drone
    std::cout << "Connecting to drone..." << std::endl;
    Robots::BetaflightUAV drone(device, baudrate);

    // Start streaming status data from drone
    std::this_thread::sleep_for(0.1ms);
    drone.subscribe();
    std::this_thread::sleep_for(0.1s);

    // Start reading drone pose from Vicon system
    Vicon::UDPClient<Vicon::ObjectDataVelocity> vicon(51001);
    while (vicon.getNumObjects() == 0) {
        std::cout << "Waiting for object" << std::endl;
        std::this_thread::sleep_for(1s);
    }

    // limits for the VICON lab in Sussex
    const Vicon::UAVControl::Bounds roomBounds{
        { -1.5_m, 1_m }, { -1_m, 1_m }, { 0_m, 2_m }
    };

    // limits for the VICON lab in Sheffield
    // const Vicon::UAVControl::Bounds roomBounds{
    //     { -2.2_m, 2.6_m }, { -4.2_m, 3.4_m }, { 0_m, 2_m }
    // };

    // Object for controlling the drone's position with the Vicon system
    Vicon::UAVControl droneControl(roomBounds);

    bool mute = true;
    bool controlOn = false;
    bool viconTimedOut = false;

    // Print initial status
    printStatus(drone, vicon);

    degree_t yaw = 0_deg;
    Vector3<meter_t> waypoint{ 0_m, 0_m, 0.2_m };
    do {
        js.update();

        // Move waypoint around
        waypoint[0] = std::max(roomBounds.x.first, std::min(roomBounds.x.second, waypoint[0] + meter_t{ speed * js.getState(HID::JAxis::RightStickHorizontal) }));
        waypoint[1] = std::max(roomBounds.y.first, std::min(roomBounds.y.second, waypoint[1] - meter_t{ speed * js.getState(HID::JAxis::RightStickVertical) }));
        waypoint[2] = std::max(roomBounds.z.first, std::min(roomBounds.z.second, waypoint[2] - meter_t{ speed * js.getState(HID::JAxis::LeftStickVertical) }));
        yaw = std::max(-180_deg, std::min(180_deg, yaw + degree_t{ js.getState(HID::JAxis::LeftStickHorizontal) }));

        if (js.isPressed(HID::JButton::A)) {
            std::cout << "Turning on control and arming" << std::endl;
            drone.arm();
        }

        if (js.isPressed(HID::JButton::B)) {
            std::cout << "Landing" << std::endl;
            waypoint = { 0_m, 0_m, 0_m };
        }

        if (js.isPressed(HID::JButton::RB)) {
            controlOn = true;
        }

        if (js.isPressed(HID::JButton::LB)) {
            std::cout << "Turning off control and disarming" << std::endl;
            drone.disarm();
            controlOn = false;
        }

        if (js.isPressed(HID::JButton::Y)) {
            mute = !mute;
        }

        if (!mute) {
            std::cout << "target:" << waypoint[0] << ", " << waypoint[1] << ", " << waypoint[2] << std::endl;
            std::cout << "yaw:" << yaw << std::endl;
            printStatus(drone, vicon);
        }

        if (controlOn) {
            const auto objectData = vicon.getObjectData(0);
            if (objectData.timeSinceReceived() > 250ms) {
                if (!viconTimedOut) {
                    drone.stopMoving();
                    std::cerr << "WARNING: Contact lost with Vicon system" << std::endl;
                    viconTimedOut = true;
                }
            } else {
                droneControl.setPose(drone, objectData, waypoint, yaw);
                if (viconTimedOut) {
                    std::cout << "Re-established contact with Vicon system" << std::endl;
                    viconTimedOut = false;
                }
            }
        }

        // We have to manually send commands to Betaflight drone
        drone.sendCommands();

        // wait so we do not overload the drone
        std::this_thread::sleep_for(10ms);
    } while (!js.isPressed(HID::JButton::X));

    drone.disarm();
}
