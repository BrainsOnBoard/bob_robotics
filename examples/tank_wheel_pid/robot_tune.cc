// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "robots/tank_netsink.h"
#include "robots/tank_pid.h"
#include "vicon/udp.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

using namespace std::literals;
using namespace BoBRobotics;
using namespace units::velocity;
using namespace units::length;
using namespace units::angular_velocity;
using namespace units::angle;

int
bob_main(int, char **)
{
    HID::Joystick joystick(0.25f);

    Vicon::UDPClient<> vicon(51001);
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    // Make connection to robot on default port
    Net::Client client;

    Robots::TankNetSink robot(client);
    robot.controlWithThumbsticks(joystick);

    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    client.runInBackground();

    Stopwatch printTimer, viconTimer;
    printTimer.start();
    viconTimer.start();
    Pose3<millimeter_t, radian_t> lastPose;
    do {
        catcher.check();

        joystick.update();

        const auto elapsed = printTimer.elapsed();
        if (elapsed > 250ms) {
            printTimer.start();

            const auto currentPose = vicon.getObjectData(0).getPose();
            const auto elapsedVicon = viconTimer.lap();
            meters_per_second_t velocity, velocityLeft, velocityRight;
            degrees_per_second_t angularVelocity;
            Vector2<meter_t> centre;
            meter_t radius;
            std::tie(velocity, velocityLeft, velocityRight, angularVelocity, centre, radius) = Robots::getVelocities(currentPose, lastPose, elapsedVicon, robot.getRobotWidth());
            lastPose = currentPose;

            // std::cout << std::fixed << std::setfill('+') << std::setw(7)
            //           << std::setprecision(4);
            std::cout << velocity << " (";
            // std::cout << std::fixed << std::setfill('+') << std::setw(7)
            //           << std::setprecision(4);
            std::cout << velocityLeft << " | ";
            // std::cout << std::fixed << std::setfill('+') << std::setw(7)
            //           << std::setprecision(4);
            std::cout << velocityRight << ")" << std::endl;
        }

        std::this_thread::sleep_for(10ms);
    } while (!joystick.isPressed(HID::JButton::B));

    return EXIT_SUCCESS;
}
