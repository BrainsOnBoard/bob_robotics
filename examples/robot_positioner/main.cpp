// C++ includes
#include <chrono>
#include <cstdlib>
#include <iostream>

#include "../../robots/robotPositioner.h"
#include "viconDataStreamer.h"
#include "../../third_party/units.h"
#include "../../robots/norbot.h"

// a small example program demonstratings the usage of 'robotPositioner.h'


int main() {

	//setup the vicon streamer
	ViconDataStreamer streamer(51001, "192.168.1.100", "c:\\users\\ad374\\Desktop", 3003);
	BoBRobotics::Robots::Norbot bot;

	// setup parameters
    millimeter_t stopping_distance   = 5_cm;                          // if the robot's distance from goal < stopping dist, robot stops
    degree_t allowed_heading_error   = 5_deg;                         // the amount of error allowed in the final heading
    double k1 = 2;                                                    // curveness of the path to the goal
    double k2 = 3;                                                    // speed of turning on the curves
    double alpha = 0.4;                                               // causes more sharply peaked curves 
    double beta  = 1.3;                                               // causes to drop velocity if 'k'(curveness) increases
    meters_per_second_t max_velocity = meters_per_second_t(0.05);     // will limit the maximum velocity to this value

    // construct the positioner
	BoBRobotics::Robots::RobotPositioner robp(
			bot,
            stopping_distance,
            allowed_heading_error,
            k1,
            k2,
            alpha,
            beta,
            max_velocity);

	// set goal pose
	robp.setGoalPose(0_mm, 0_mm, 15_deg);

	while (!robp.didReachGoal()) {
		BoBRobotics::Vector3<millimeter_t> pos_robot = streamer.getTranslation();
		BoBRobotics::Vector3<radian_t>     rot_robot = streamer.getRotation();

		millimeter_t posx = pos_robot[0];
		millimeter_t posy = pos_robot[1];
		degree_t heading  = rot_robot[0];

		robp.updateMotors(posx,posy,heading);

	}
	
        
	return 0;
}

