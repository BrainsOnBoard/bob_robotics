// BoB robotics includes
#include "common/logging.h"
#include "common/macros.h"
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "robots/tank_netsink.h"
#include "vicon/udp.h"

// Standard C++ includes
#include <chrono>
#include <fstream>
#include <thread>

using namespace BoBRobotics;
using namespace units::literals;
using namespace units::time;
using namespace std::literals;

int
bob_main(int argc, char **argv)
{
    std::string ipAddress;
    if (argc > 1) {
        ipAddress = argv[1];
    } else {
        LOGI << "IP address [10.0.0.4]: ";
        std::getline(std::cin, ipAddress);
        if (ipAddress.empty()) {
            ipAddress = "10.0.0.4";
        }
    }

    // Connect to robot
    LOGI << "Connecting to robot";
    Net::Client client(ipAddress);
    LOGI << "Connected to " << ipAddress;

    // Send motor commands to robot
    Robots::TankNetSink robot(client);

    // Open joystick
    HID::Joystick joystick;
    robot.addJoystick(joystick);
    LOGI << "Opened joystick";

    Stopwatch stopwatch;
    std::ofstream dataFile;
    Vicon::UDPClient<Vicon::ObjectDataVelocity> vicon(51001); // For getting robot's position

    // If we're recording, ignore axis movements
    joystick.addHandler([&stopwatch](HID::JAxis, float) {
        if (stopwatch.started()) {
            LOGI << "Ignoring joystick command";
            return true;
        } else {
            return false;
        }
    });

    // Toggle testing mode with buttons
    joystick.addHandler([&](HID::JButton button, bool pressed) {
        if (!pressed) {
            return false;
        }

        if (button == HID::JButton::Y) {
            // Start recording
            if (!stopwatch.started()) {
                // Open a new file for writing
                dataFile.open("data/angular_velocity.csv");
                BOB_ASSERT(dataFile.good());
                dataFile << "Time [ms], Yaw [rad], Pitch [rad], Roll [rad], "
                         << "Yaw velocity [rad/s], Pitch velocity [rad/s], Roll velocity [rad/s]"
                        ;

                // Spin robot
                stopwatch.start();
                robot.tank(1.f, -1.f);

                LOGI << "Started recording";
            }
            return true;
        } else if (button == HID::JButton::X) {
            // Stop recording
            if (stopwatch.started()) {
                robot.stopMoving();
                stopwatch.reset();
                dataFile.close();
                LOGI << "Stopped recording";
            }
            return true;
        } else {
            return false;
        }
    });

    // Run client in background, checking for background errors thrown
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch ctrl-C
    client.runInBackground();
    while (!joystick.isPressed(HID::JButton::B)) {
        // Check for errors
        catcher.check();

        // Poll joystick for events
        joystick.update();

        if (stopwatch.started()) { // If we're recording
            const millisecond_t time = stopwatch.elapsed();
            const auto objectData = vicon.getObjectData();
            const auto attitude = objectData.getPose().attitude();
            const auto angvel = objectData.getAngularVelocity();
            dataFile << time() << ", " << attitude[0]() << ", " << attitude[1]() << ", " << attitude[2]() << ", "
                     << angvel[0]() << ", " << angvel[1]() << ", " << angvel[2]() << "\n";
        }

        std::this_thread::sleep_for(20ms);
    }

    return EXIT_SUCCESS;
}
