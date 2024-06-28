// BoB robotics includes
#include "plog/Log.h"
#include "hid/net/source.h"
#include "net/server.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics::HID;

int bobMain(int, char **)
{
    // Listen for incoming connection on default port
    BoBRobotics::Net::Server server;
    auto connection = server.waitForConnection();

    // Read motor commands from network
    Net::Source js(*connection);

    for (int i = 1; !js.isDown(JButton::B); i++) {
        // read from joystick
        js.update();

        if (js.isDown(JButton::A)) {
            LOGI << i << " | A is down";
        }
        if (js.isPressed(JButton::A)) {
            LOGI << i << " | A has been pressed";
        }
        if (js.isReleased(JButton::A)) {
            LOGI << i << " | A has been released";
        }

        // wait
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return EXIT_SUCCESS;
}
