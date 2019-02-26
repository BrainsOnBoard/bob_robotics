// BoB robotics includes
#include "common/main.h"
#include "common/pose.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "robots/robot_positioner_control.h"
#include "robots/tank_netsink.h"

// Third-party includes
#include "third_party/units.h"

using namespace BoBRobotics;
using namespace units::length;
using namespace units::angle;
using namespace units::literals;

int
bob_main(int, char **)
{
    HID::Joystick joystick;

    // Make connection to robot on default port
    Net::Client client;

    // Connect to robot over network
    Robots::TankNetSink tank(client);

    // Positioner parameters
    constexpr meter_t StoppingDistance = 10_cm;     // if the robot's distance from goal < stopping dist, robot stops
    constexpr radian_t AllowedHeadingError = 5_deg; // the amount of error allowed in the final heading
    constexpr double K1 = 0.2;                      // curveness of the path to the goal
    constexpr double K2 = 5;                        // speed of turning on the curves
    constexpr double Alpha = 1.03;                  // causes more sharply peaked curves
    constexpr double Beta = 0.02;                   // causes to drop velocity if 'k'(curveness) increases
    const Pose2<millimeter_t, degree_t> Goal{ 0_mm, 0_mm, 15_deg };

    // Object which drives robot to position
    Robots::RobotPositioner positioner(StoppingDistance,
                                       AllowedHeadingError,
                                       K1,
                                       K2,
                                       Alpha,
                                       Beta);

    // Handles joystick, Vicon system etc.
    Robots::RobotPositionerControl control(tank, positioner, joystick);
    control.setGoalPose(Goal);
    control.run();

    return EXIT_SUCCESS;
}