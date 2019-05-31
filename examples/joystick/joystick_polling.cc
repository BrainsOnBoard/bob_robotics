// BoB robotics includes
#include "hid/joystick.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
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
            std::cout << i << " | A is down" << std::endl;
        }
        if (js.isPressed(JButton::A)) {
            std::cout << i << " | A has been pressed" << std::endl;
        }
        if (js.isReleased(JButton::A)) {
            std::cout << i << " | A has been released" << std::endl;
        }

        // wait
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}