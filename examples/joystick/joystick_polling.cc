// BoB robotics includes
#include "common/logging.h"
#include "hid/joystick.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics::HID;

int
main()
{
    Joystick js;
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
}