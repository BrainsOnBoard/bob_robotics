// BoB robotics includes
#include "common/logging.h"
#include "common/pose.h"
#include "robots/control/robot_positioner.h"
#include "robots/simulated_tank.h"
#include "viz/sfml_world/sfml_world.h"

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
    Viz::SFMLWorld display;
    auto car = display.createCarAgent();

    // A circle to show where the goal is
    sf::CircleShape goalCircle(10);
    goalCircle.setFillColor(sf::Color::Blue);
    goalCircle.setOrigin(10, 10);
    goalCircle.setPosition(Viz::SFMLWorld::WindowWidth / 2, Viz::SFMLWorld::WindowHeight / 2);

    constexpr meter_t stoppingDistance = 5_cm;      // if the robot's distance from goal < stopping dist, robot stops
    constexpr radian_t allowedHeadingError = 2_deg; // the amount of error allowed in the final heading
    constexpr double k1 = 1.51;                     // curveness of the path to the goal
    constexpr double k2 = 4.4;                      // speed of turning on the curves
    constexpr double alpha = 1.03;                  // causes more sharply peaked curves
    constexpr double beta = 0.02;                   // causes to drop velocity if 'k'(curveness) increases

    // construct the positioner
    auto positioner = Robots::createRobotPositioner(
            robot,
            robot,
            stoppingDistance,
            allowedHeadingError,
            k1,
            k2,
            alpha,
            beta);

    bool runPositioner = false;
    while (display.isOpen()) {
        // Run GUI events
        car.setPose(robot.getPose());
        const sf::Event event = display.updateAndDrive(robot, goalCircle, car);

        // Spacebar toggles whether positioner is running
        if (event.type == sf::Event::KeyReleased && event.key.code == sf::Keyboard::Space) {
            runPositioner = !runPositioner;
            robot.stopMoving();
        }

        if (runPositioner) {
            if (display.mouseClicked()) {
                // Set a new goal position if user clicks in the window
                const auto mousePosition = display.mouseClickPosition();

                // Set the goal to this position
                positioner.moveTo({ mousePosition.x(), mousePosition.y(), 15_deg });

                goalCircle.setPosition(display.vectorToPixel(mousePosition));
            }

            // Check if the robot is within threshold distance and bearing of goal
            if (!positioner.pollPositioner()) {
                runPositioner = false;
                LOGI << "Reached goal";
                robot.stopMoving();
            }
        }

        // A small delay, so we don't eat all the CPU
        std::this_thread::sleep_for(2ms);
    }
}
