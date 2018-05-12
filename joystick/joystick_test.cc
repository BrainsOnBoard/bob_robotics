#include "joystick.h"
#include <string>

Joystick::Joystick joystick;

void
handleButton(unsigned int number, int16_t value)
{
    std::string name = joystick.getButtonName(number);
    if (value) {
        std::cout << "Button pushed: " << name << " (" << (int) number << ")"
                  << std::endl;
    } else {
        std::cout << "Button released: " << name << " (" << (int) number << ")"
                  << std::endl;
    }
}

void
handleAxis(unsigned int number, int16_t value)
{
    std::string name = joystick.getAxisName(number);
    std::cout << "Axis " << name << " (" << (int) number << "): " << value
              << std::endl;
}

void
callback(Joystick::Event *js, void *)
{
    if (!js) {
        std::cerr << "Error reading from joystick" << std::endl;
        exit(1);
    }

    if (js->isAxis) {
        handleAxis(js->number, js->value);
    } else {
        handleButton(js->number, js->value);
    }
}

int
main()
{
    std::cout << "Joystick test program" << std::endl;
    std::cout << "Press return to quit" << std::endl << std::endl;

    if (!joystick.open()) {
        std::cerr << "Error: Could not open joystick" << std::endl;
        return 1;
    }

    std::cout << "Opened joystick" << std::endl;
    joystick.startThread(callback, nullptr);

    std::cin.ignore();

    joystick.close();
    std::cout << "Controller closed" << std::endl;

    return 0;
}
