// BoB robotics includes
#include "common/pose.h"
#include "robots/control/obstacle_circumnavigation.h"
#include "robots/control/robot_positioner.h"
#include "robots/tank/simulated_tank.h"
#include "viz/sfml/robot_control.h"
#include "viz/sfml/world.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/units.h"

using namespace BoBRobotics;
using namespace units::length;
using namespace units::velocity;
using namespace units::angular_velocity;
using namespace units::time;
using namespace units::literals;
using namespace units::angle;

int bobMain(int, char **)
{
    Robots::Tank::SimulatedTank<> robot(0.3_mps, 104_mm);
    Viz::SFML::World display;
    auto car = display.createCarAgent();

    // A circle to show where the goal is
    sf::CircleShape goalCircle(10);
    goalCircle.setFillColor(sf::Color::Blue);
    goalCircle.setOrigin(10, 10);
    goalCircle.setPosition(Viz::SFML::World::WindowWidth / 2, Viz::SFML::World::WindowHeight / 2);

    constexpr meter_t stoppingDistance = 5_cm;      // if the robot's distance from goal < stopping dist, robot stops
    constexpr radian_t allowedHeadingError = 2_deg; // the amount of error allowed in the final heading
    constexpr double k1 = 1.51;                     // curveness of the path to the goal
    constexpr double k2 = 4.4;                      // speed of turning on the curves
    constexpr double alpha = 1.03;                  // causes more sharply peaked curves
    constexpr double beta = 0.02;                   // causes to drop velocity if 'k'(curveness) increases

    // Construct the positioner
    auto positioner = Robots::createRobotPositioner(
            robot,
            robot,
            stoppingDistance,
            allowedHeadingError,
            k1,
            k2,
            alpha,
            beta);

    // The x and y dimensions of the robot
    const auto halfWidth = car.getSize().x() / 2;
    const auto halfLength = car.getSize().y() / 2;
    using V = Vector2<meter_t>;
    const std::array<V, 4> robotDimensions = {
        V{ -halfWidth, halfLength },
        V{ halfWidth, halfLength },
        V{ halfWidth, -halfLength },
        V{ -halfWidth, -halfLength }
    };

    const std::vector<std::vector<V>> objects = {
        {
              V{ -30_cm, -30_cm },
              V{ -40_cm, -30_cm },
              V{ -40_cm, -40_cm },
              V{ -30_cm, -40_cm }
         } };

    const auto size = display.lengthToPixel(10_cm);
    sf::RectangleShape objectShape({ size, size });
    const auto offset = display.lengthToPixel(5_cm);
    objectShape.setOrigin(offset, offset);
    objectShape.setPosition(display.vectorToPixel(Vector2<meter_t>{ -35_cm, -35_cm }));
    objectShape.setFillColor(sf::Color::Black);

    // Objects for controlling circumnavigation
    Robots::CollisionDetector collisionDetector{ robotDimensions, objects, 5_cm, 1_cm };
    auto circum = createObstacleCircumnavigator(robot, robot, collisionDetector);
    auto avoidingPositioner = createObstacleAvoidingPositioner(positioner, circum);

    bool runPositioner = false;
    while (display.isOpen()) {
        car.setPose(robot.getPose());

        auto eventHandler = [&](const sf::Event &event) {
            // drive robot with keyboard
            if (Viz::SFML::drive(robot, event)) {
                return;
            }

            // Spacebar toggles whether positioner is running
            if (event.type == sf::Event::KeyReleased && event.key.code == sf::Keyboard::Space) {
                runPositioner = !runPositioner;
                robot.stopMoving();
                return;
            }

            if (runPositioner && event.type == sf::Event::MouseButtonPressed
                && event.mouseButton.button == sf::Mouse::Left) {

                // set a new goal position if user clicks in the window
                const auto vec = display.pixelToVector(event.mouseButton.x, event.mouseButton.y);

                // Set the goal to this position
                const Pose2<meter_t, radian_t> goalPose{ vec.x(), vec.y(), 15_deg };
                avoidingPositioner.moveTo(goalPose);

                goalCircle.setPosition(event.mouseButton.x, event.mouseButton.y);
            }
        };

        // Run GUI events
        display.drawAndHandleEvents(eventHandler, objectShape, goalCircle, car);

        // Check if the robot is within threshold distance and bearing of goal
        if (runPositioner && !avoidingPositioner.pollPositioner()) {
            runPositioner = false;
            LOGI << "Reached goal";
            robot.stopMoving();
        }
    }
    return EXIT_SUCCESS;
}
