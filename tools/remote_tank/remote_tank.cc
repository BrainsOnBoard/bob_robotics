// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "hid/joystick.h"
#include "hid/robot_control.h"
#include "net/client.h"
#include "robots/tank/net/sink.h"

// Standard C++ includes
#include <chrono>
#include <exception>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;

int bobMain(int, char **)
{
    // Enable networking on Windows
    OS::Net::WindowsNetworking::initialise();

    // Make connection to robot on default port
    Net::Client client;

    // Transmit motor commands over network
    Robots::Tank::Net::Sink tank(client);

    // Run client on background thread, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    client.runInBackground();

    // Add joystick for controlling robot
    HID::Joystick joystick;
    HID::addJoystick(tank, joystick);

    while (!joystick.isPressed(HID::JButton::B)) {
        // Rethrow any exceptions caught on background thread
        catcher.check();

        // Poll joystick
        joystick.update();

        std::this_thread::sleep_for(100ms);
    }

    return EXIT_SUCCESS;
}
