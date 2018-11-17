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

auto
now()
{
    return std::chrono::high_resolution_clock::now();
}

int
main()
{
    Robots::Simulator sim(50_cm, 0.1_mps, 34_mm);

    millimeter_t stopping_distance = 5_cm;  // if the robot's distance from goal < stopping dist, robot stops
    degree_t allowed_heading_error = 1_deg; // the amount of error allowed in the final heading
    double k1 = 1.51;                       // curveness of the path to the goal
    double k2 = 4.4;                        // speed of turning on the curves
    double alpha = 1.03;                    // causes more sharply peaked curves
    double beta = 0.02;                     // causes to drop velocity if 'k'(curveness) increases
    meters_per_second_t max_velocity{ 1 };  // will limit the maximum velocity to this value
    degrees_per_second_t max_turning_velocity{ 50 };

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

    bool runPositioner = false;
    auto lastTime = now();
    while (!sim.didQuit()) {
        const auto currentTime = now();
        const auto &pose = sim.getPose();

        if (runPositioner) {
            // setting a new goal position if user clicks in the window
            const auto mousePosition = sim.getMouseClickLocation();

            // change the coordinates to mm
            robp.setGoalPose({ mousePosition[0], mousePosition[1], 15_deg });

            // update course and drive robot
            robp.updateMotors(sim, pose);
        }

        if (sim.simulationStep() == SDLK_SPACE) {
            runPositioner = !runPositioner;
            if (runPositioner) {
                std::cout << "Starting simulation" << std::endl;
            } else {
                std::cout << "Stopping simulation" << std::endl;
                sim.stopMoving();

                // Reset agent's position
                sim.setPose(Pose2<millimeter_t, degree_t>{});
            }
        }
        lastTime = currentTime;

        std::this_thread::sleep_for(10ms);
    }
}
