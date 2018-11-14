// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>

#include "../../robots/norbot.h"
#include "../../robots/robot_positioner.h"
#include "../../third_party/units.h"
#include "viconDataStreamer.h"

// a small example program demonstratings the usage of 'robotPositioner.h'

using namespace units::length;
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::velocity;

int
main()
{

    //setup the vicon streamer
    ViconDataStreamer streamer(51001, "192.168.1.100", "c:\\users\\ad374\\Desktop", 3003);
    BoBRobotics::Robots::Norbot bot;

    // setup parameters
    constexpr millimeter_t stopping_distance = 10_cm;   // if the robot's distance from goal < stopping dist, robot stops
    constexpr degree_t allowed_heading_error = 5_deg;   // the amount of error allowed in the final heading
    constexpr double k1 = 1;                            // curveness of the path to the goal
    constexpr double k2 = 2;                            // speed of turning on the curves
    constexpr double alpha = 1.4;                       // causes more sharply peaked curves
    constexpr double beta = 0.3;                        // causes to drop velocity if 'k'(curveness) increases
    constexpr meters_per_second_t max_velocity{ 0.05 }; // will limit the maximum velocity to this value
    constexpr degrees_per_second_t max_turning_velocity{ 13 };

    BoBRobotics::Robots::RobotPositioner robp(
            stopping_distance,
            allowed_heading_error,
            k1,
            k2,
            alpha,
            beta,
            max_velocity,
            max_turning_velocity);

    // set goal pose
    robp.setGoalPose(0_mm, 0_mm, 15_deg);

    while (1) {
        BoBRobotics::Vector3<millimeter_t> pos_robot = streamer.getTranslation();
        BoBRobotics::Vector3<radian_t> rot_robot = streamer.getRotation();

        millimeter_t posx = pos_robot[0];
        millimeter_t posy = pos_robot[1];
        degree_t heading = rot_robot[0];

        meters_per_second_t v;
        degrees_per_second_t w;
        robp.updateMotors(bot, posx, posy, heading);
    }

    return 0;
}
