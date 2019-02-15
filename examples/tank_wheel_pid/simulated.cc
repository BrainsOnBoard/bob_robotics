#include "common.h"

// BoB robotics includes
#include "robots/simulated_tank.h"

int
bob_main(int, char **)
{
    Robots::TankPID<Robots::SimulatedTank<>> robot(1.f, 0.f, 0.f, 0.3_mps, 104_mm);
    robot.updatePose(Pose2<meter_t, radian_t>{}, 0_s);

    HID::Joystick joystick(0.25f);
    joystick.addHandler([&robot](HID::JButton button, bool pressed) {
        if (pressed && button == HID::JButton::Start) {
            robot.setPose({}); // Reset to origin
            return true;
        } else {
            return false;
        }
    });

    runWheelPID(joystick, robot, robot);

    return EXIT_SUCCESS;
}
