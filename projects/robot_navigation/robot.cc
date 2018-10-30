// Windows headers
#include "os/windows_include.h"

// If we're compiling on Windows, we know we don't need I2C
#ifdef _WIN32
#define NO_I2C_ROBOT
#endif

// BoB robotics includes
#include "hid/joystick.h"
#include "robots/tank.h"
#include "video/panoramic.h"

#ifndef NO_I2C_ROBOT
#include "robots/norbot.h"
#endif

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace std::literals;
using namespace BoBRobotics;

int
main()
{
    try {
        auto cam = Video::getPanoramicCamera();

#ifdef NO_I2C_ROBOT
        // Output motor commands to terminal
        Robots::Tank tank;
#else
        // Use Arduino robot
        Robots::Norbot tank;
#endif

        // Control robot with joystick
        HID::Joystick joystick;
        tank.addJoystick(joystick);

        // Poll joystick
        while (!joystick.isPressed(HID::JButton::B)) {
            if (!joystick.update()) {
                std::this_thread::sleep_for(10ms);
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
        return 1;
    }
}
