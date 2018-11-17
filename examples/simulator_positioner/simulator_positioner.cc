// BoB robotics includes
#include "common/pose.h"
#include "robots/robot_positioner.h"
#include "robots/simulator.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace units::length;
using namespace units::velocity;
using namespace units::angular_velocity;
using namespace units::time;
using namespace units::literals;
using namespace units::angle;
using namespace std::literals;

int
main()
{
    Robots::Simulator sim(50_cm, 0.1_mps, 34_mm);

    constexpr meter_t stoppingDistance = 5_cm;      // if the robot's distance from goal < stopping dist, robot stops
    constexpr radian_t allowedHeadingError = 1_deg; // the amount of error allowed in the final heading
    constexpr double k1 = 1.51;                     // curveness of the path to the goal
    constexpr double k2 = 4.4;                      // speed of turning on the curves
    constexpr double alpha = 1.03;                  // causes more sharply peaked curves
    constexpr double beta = 0.02;                   // causes to drop velocity if 'k'(curveness) increases

    // construct the positioner
    Robots::RobotPositioner robp(
            stoppingDistance,
            allowedHeadingError,
            k1,
            k2,
            alpha,
            beta,
            sim.getMaximumSpeed(),
            sim.getMaximumTurnSpeed());

    bool runPositioner = false;
    bool reachedGoalAnnounced = false;
    while (sim.isOpen()) {
        const auto &pose = sim.getPose();

        if (runPositioner) {
            // setting a new goal position if user clicks in the window
            const auto mousePosition = sim.getMouseClickPosition();

            // set the goal to this position
            robp.setGoalPose({ mousePosition[0], mousePosition[1], 15_deg });

            // update course and drive robot
            robp.updateMotors(sim, pose);

            // check if the robot is within threshold distance and bearing of goal
            if (robp.didReachGoal()) {
                if (!reachedGoalAnnounced) {
                    std::cout << "Reached goal" << std::endl;
                    reachedGoalAnnounced = true;
                }
            } else {
                reachedGoalAnnounced = false;
            }
        }

        // start/stop homing when spacebar is pressed
        if (sim.simulationStep() == SDLK_SPACE) {
            runPositioner = !runPositioner;
            if (runPositioner) {
                std::cout << "Starting simulation" << std::endl;
            } else {
                std::cout << "Stopping simulation" << std::endl;
                sim.stopMoving();

                // Reset agent's position
                sim.setPose({});
            }
        }

        std::this_thread::sleep_for(10ms);
    }
}
