#include "robots/bebop.h"

using namespace GeNNRobotics;

int main()
{
    const float deadZone = 0.25f;
    HID::Joystick joystick(deadZone);

    Robots::Bebop drone;
    drone.connect();
    drone.setFlightEventHandler([&joystick](bool takeoff) {
        if (!takeoff) {
            joystick.stop();
        }
    });
    drone.addJoystick(joystick);

    joystick.run();
}
