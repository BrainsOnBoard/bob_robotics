// BoB robotics includes
#include "robots/robotPositioner.h"
#include "robots/simulator.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>

using namespace BoBRobotics;
using namespace units::length;
using namespace units::velocity;
using namespace units::angular_velocity;
using namespace units::time;
using namespace units::literals;
using namespace units::angle;

int
main()
{
    Robots::Simulator sim;
    sim.setRobotSize(16.4_cm, 35_cm);

    millimeter_t stopping_distance = 5_cm;                     // if the robot's distance from goal < stopping dist, robot stops
    degree_t allowed_heading_error = 1_deg;                    // the amount of error allowed in the final heading
    double k1 = 1.51;                                          // curveness of the path to the goal
    double k2 = 4.4;                                           // speed of turning on the curves
    double alpha = 1.03;                                       // causes more sharply peaked curves
    double beta = 0.02;                                        // causes to drop velocity if 'k'(curveness) increases
    meters_per_second_t max_velocity = meters_per_second_t(5); // will limit the maximum velocity to this value
    degrees_per_second_t max_turning_velocity = degrees_per_second_t(90);

    // construct the positioner
    Robots::RobotPositioner robp(
            stopping_distance,
            allowed_heading_error,
            k1,
            k2,
            alpha,
            beta,
            max_velocity,
            max_turning_velocity);

    robp.setGoalPose(millimeter_t(1200), millimeter_t(1600), 15_deg);

    while (!sim.didQuit()) {

        // timing start
        auto t1 = std::chrono::high_resolution_clock::now();

        meters_per_second_t v;
        degrees_per_second_t w;

        std::vector<float> pose = sim.getCurrentPosition();

        // setting a new goal position if user clicks in the window
        std::vector<float> mouseClickedPosition = sim.getMouseClickLocation();
        float newPositionX = mouseClickedPosition.at(0);
        float newPositionY = mouseClickedPosition.at(1);

        // change the coordinates to mm
        millimeter_t nPosX, nPosY;
        Robots::Simulator::changePixelToMM(newPositionX, newPositionY, nPosX, nPosY);
        robp.setGoalPose(nPosX, nPosY, 15_deg);

        // change the robot's coordinates to real units (mm)
        millimeter_t robotPoseX, robotPoseY;
        Robots::Simulator::changePixelToMM(pose[0], pose[1], robotPoseX, robotPoseY);

        if (!robp.didReachGoal()) {
            robp.updateVelocities(robotPoseX, robotPoseY, degree_t(pose[2]), v, w);
        } else {
            v = meters_per_second_t(0);
            w = degrees_per_second_t(0);
        }

        //timing end
        auto t2 = std::chrono::high_resolution_clock::now();
        sim.simulationStep(v, w, t2 - t1);
    }
}
