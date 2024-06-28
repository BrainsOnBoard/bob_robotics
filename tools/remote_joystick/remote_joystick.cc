// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "hid/joystick.h"
#include "hid/net/sink.h"
#include "net/client.h"

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

    // Create joystick
    HID::Joystick joystick;

    // Transmit motor commands over network
    HID::Net::Sink sink(client, joystick);

    // Run client on background thread, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    client.runInBackground();

    while (!joystick.isPressed(HID::JButton::X)) {
        // Rethrow any exceptions caught on background thread
        catcher.check();

        // Poll joystick
        joystick.update();

        std::this_thread::sleep_for(100ms);
    }

    return EXIT_SUCCESS;
}
