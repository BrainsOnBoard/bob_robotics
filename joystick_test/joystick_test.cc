#include "../common/joystick.h"

int main()
{
    constexpr float deadzone = 0.25f;
    Joystick test;

    while(true) {
        const float x = test.getAxisState(0);
        const float y = test.getAxisState(1);
        if(x < -deadzone) {
            std::cout << "Left" << std::endl;
        }
        else if(x > deadzone) {
            std::cout << "Right" << std::endl;
        }
        else if(y < -deadzone) {
            std::cout << "Forwards" << std::endl;
        }
        else if(y > deadzone) {
            std::cout << "Back" << std::endl;
        }

    }
    return 0;
}