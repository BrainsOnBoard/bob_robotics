// BoB robotics includes
#include "common/background_exception_catcher.h"
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

    // While TCP/IP connection remains open
    BoBRobotics::BackgroundExceptionCatcher catcher;
    catcher.trapSignals();
    connection->runInBackground();
    for (int i = 1; connection->isOpen(); i++) {
        // Rethrow any exceptions caught on background thread
        catcher.check();

        // read from joystick
        js.update();

        LOGI << js.getState(JAxis::LeftStickHorizontal);
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
