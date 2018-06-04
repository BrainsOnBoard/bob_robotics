// C++ includes
#include <iostream>
#include <string>

// local includes
#include "joystick.h"

using namespace GeNNRobotics::HID;

bool
onAxisEvent(JAxis axis, float value)
{
    std::string name = Joystick::getName(axis);
    std::cout << "Axis " << name << " (" << static_cast<int>(axis) << "): "
              << value << std::endl;
    
    // we handle all axis events
    return true;
}

bool
onButtonEvent(JButton button, bool pressed)
{
    std::string name = Joystick::getName(button);
    std::cout << "Button " << name << " (" << static_cast<int>(button)
              << (pressed ? ") pushed" : ") released") << std::endl;

    // we handle all button events
    return true;
}

int
main()
{
    std::cout << "Joystick test program" << std::endl;
    std::cout << "Press return to quit" << std::endl << std::endl;

    Joystick joystick;
    std::cout << "Opened joystick" << std::endl;

    // get initial axis states
    for (int i = 0; i < static_cast<int>(JAxis::LENGTH); i++) {
        JAxis axis = static_cast<JAxis>(i);
        std::cout << "[initial] ";
        onAxisEvent(axis, joystick.getAxisState(axis));
    }

    // add handlers for button and axis events
    joystick.addHandler(onAxisEvent);
    joystick.addHandler(onButtonEvent);
    joystick.runInBackground();

    // wait until keypress
    std::cin.ignore();
    return 0;
}
