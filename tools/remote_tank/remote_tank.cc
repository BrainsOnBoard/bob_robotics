// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "robots/tank_netsink.h"

// Standard C++ includes
#include <chrono>
#include <exception>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;

int
bob_main(int argc, char **argv)
{
    std::string robotIP;
    if (argc == 2) {
        // Get robot IP from command-line argument
        robotIP = argv[1];
    } else {
        // Get robot IP from terminal
        std::cout << "Robot IP [127.0.0.1]: ";
        std::getline(std::cin, robotIP);
        if (robotIP.empty()) {
            robotIP = "127.0.0.1";
        }
    }

    // Make connection to robot on default port
    Net::Client client(robotIP);

    // Run client on background thread, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    client.runInBackground();

    // Transmit motor commands over network
    Robots::TankNetSink tank(client);

    // Add joystick for controlling robot
    HID::Joystick joystick;
    tank.addJoystick(joystick);

    while (!joystick.isPressed(HID::JButton::B)) {
        // Rethrow any exceptions caught on background thread
        catcher.check();

        // Poll joystick
        joystick.update();
        std::this_thread::sleep_for(20ms);
    }

    return EXIT_SUCCESS;
}
