#include "robots/bebop.h"

using namespace GeNNRobotics;

int main()
{
    HID::Joystick joystick(/*deadZone=*/0.25);

    Robots::Bebop drone;
    drone.connect();
    drone.setFlightEventHandler([&joystick](bool takeoff) {
        if (!takeoff) {
            joystick.stop();
        }
    });
    drone.addJoystick(joystick, /*maxSpeed=*/0.25);

    joystick.run();
}
