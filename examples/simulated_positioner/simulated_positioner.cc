// BoB robotics includes
#include "common/pose.h"
#include "robots/robot_positioner.h"
#include "robots/simulated_tank.h"
#include "robots/car_display.h"

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
    Robots::SimulatedTank<> robot(0.3_mps, 104_mm);
    Robots::CarDisplay display;

    constexpr meter_t stoppingDistance = 5_cm;      // if the robot's distance from goal < stopping dist, robot stops
    constexpr radian_t allowedHeadingError = 2_deg; // the amount of error allowed in the final heading
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
            robot.getMaximumSpeed(),
            robot.getMaximumTurnSpeed());

    bool runPositioner = false;
    bool reachedGoalAnnounced = false;
    while (display.isOpen()) {
        // Get the robot's current pose
        const auto &pose = robot.getPose();

        // Run GUI events, get pressed key
        const auto key = display.runGUI(pose);
        if (key.second) { // Key down
            switch (key.first) {
            case SDLK_LEFT:
                robot.tank(-0.5f, 0.5f);
                break;
            case SDLK_RIGHT:
                robot.tank(0.5f, -0.5f);
                break;
            case SDLK_UP:
                robot.tank(1.f, 1.f);
                break;
            case SDLK_DOWN:
                robot.tank(-1.f, -1.f);
                break;
            case SDLK_SPACE:
                // start/stop positioner
                runPositioner = !runPositioner;
                if (runPositioner) {
                    std::cout << "Starting simulation" << std::endl;
                } else {
                    std::cout << "Stopping simulation" << std::endl;
                    robot.stopMoving();

                    // Reset agent's position to origin
                    robot.setPose({});
                }
            }
        } else { // Key up
            switch (key.first) {
            case SDLK_LEFT:
            case SDLK_RIGHT:
            case SDLK_UP:
            case SDLK_DOWN:
                robot.stopMoving();
                break;
            }
        }

        if (runPositioner) {
            // Set a new goal position if user clicks in the window
            const auto mousePosition = display.getMouseClickPosition();

            // Set the goal to this position
            robp.setGoalPose({ mousePosition[0], mousePosition[1], 15_deg });

            // Update course and drive robot
            robp.updateMotors(robot, pose);

            // Check if the robot is within threshold distance and bearing of goal
            if (robp.reachedGoal()) {
                if (!reachedGoalAnnounced) {
                    std::cout << "Reached goal" << std::endl;
                    reachedGoalAnnounced = true;
                }
            } else {
                reachedGoalAnnounced = false;
            }
        }

        // A small delay, so we don't eat all the CPU
        std::this_thread::sleep_for(2ms);
    }
}
