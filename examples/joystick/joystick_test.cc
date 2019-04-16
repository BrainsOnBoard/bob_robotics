// BoB robotics includes
#include "hid/joystick.h"

// Standard C++ includes
#include <iostream>
#include <string>

using namespace BoBRobotics::HID;

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

inline void
initButton(Joystick &joystick, JButton button)
{
    std::cout << "[initial] ";
    onButtonEvent(button, joystick.isDown(button));
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
        onAxisEvent(axis, joystick.getState(axis));
    }

    // get initial button states
    initButton(joystick, JButton::A);
    initButton(joystick, JButton::B);
    initButton(joystick, JButton::X);
    initButton(joystick, JButton::Y);
    initButton(joystick, JButton::LB);
    initButton(joystick, JButton::RB);
    initButton(joystick, JButton::Back);
    initButton(joystick, JButton::Start);
    initButton(joystick, JButton::LeftStick);
    initButton(joystick, JButton::RightStick);

    // add handlers for button and axis events
    joystick.addHandler(onAxisEvent);
    joystick.addHandler(onButtonEvent);

    // run joystick on main thread
    joystick.run();

    return 0;
}
