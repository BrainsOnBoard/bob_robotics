// C++ includes
#include <iostream>
#include <string>

// local includes
#include "joystick.h"

using namespace GeNNRobotics;

void
callback(HID::Event &js)
{
    if (js.isInitial) {
        std::cout << "[initial] ";
    }

    if (js.isAxis) {
        std::string name = js.axisName();
        std::cout << "Axis " << name << " (" << js.number << "): "
                  << js.value << std::endl;
    } else {
        std::string name = js.buttonName();
        std::cout << "Button " << name << " (" << js.number << (js.value ? ") pushed" : ") released")
                  << std::endl;
    }
}

int
main()
{
    std::cout << "Joystick test program" << std::endl;
    std::cout << "Press return to quit" << std::endl << std::endl;

    HID::Joystick joystick(callback);
    std::cout << "Opened joystick" << std::endl;
    joystick.runInBackground();

    // wait until keypress
    std::cin.ignore();
    return 0;
}
