/*
    Small example program to demonstrate the Pure Pursuit Controller. The controller
    takes a list of coordinates as waypoint and calculates a desired turning angle
    to steer the car for a smooth path tracking.
    To create a path, press the left mouse button to place a waypoint to the screen
    making a path. Then, pressing the space button will start the controller algorithm
    to make the robot follow the path

    key commands:
        [    ARROW UP     ] = apply max speed to car
        [   ARROW DOWN    ] = stop car
        [   ARROW LEFT    ] = turn steering wheel by 30 degrees left
        [   ARROW RIGHT   ] = turn steering wheel by 30 degrees right
        [      SPACE      ] = toggle controller algorithm on/off
        [LEFT MOUSE BUTTON] = places a waypoint on the screen adding it to the path
*/

// BoB robotics includes
#include "robots/control/pure_pursuit_controller.h"
#include "robots/ackermann/simulated_ackermann.h"
#include "viz/sfml/robot_control.h"
#include "viz/sfml/world.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace units::length;
using namespace units::angle;
using namespace units::velocity;
using namespace units::math;

int
bobMain(int, char **)
{
    constexpr millimeter_t LookAheadDistance = 1_m;   // lookahead distance
    constexpr meters_per_second_t MaxSpeed = 1.4_mps; // car's max speed
    constexpr degree_t MaxTurn = 30_deg;              // car's maximum turning angle
    constexpr millimeter_t StoppingDist = 1_cm;       // car's stopping distance

    Robots::Ackermann::SimulatedAckermann robot(MaxSpeed, 500_mm, 0_m, MaxTurn); // simulated robot
    Viz::SFML::World display({ 10.2_m, 10.2_m });                                // display the agent
    auto car = display.createCarAgent(160_mm);                                   // car sprite
    bool isControllerOn = false;
    car.setPose(Pose2<meter_t, degree_t>{});

    auto wpLines = display.createLineStrip(sf::Color::Blue);      // lines between way points
    std::vector<sf::RectangleShape> wpRects;                      // rectangles representing way points
    auto lookAheadLine = display.createLineStrip(sf::Color::Red); // a line between robot and lookahead point

    const auto wheelBase = robot.getDistanceBetweenAxis(); // distance between wheel bases
    Robots::PurePursuitController controller(LookAheadDistance, wheelBase, StoppingDist);
    bool warningDisplayed = false; // has message about stuck robot already been displayed?

    auto addWayPoint = [&](const auto &position) {
        constexpr float SquareWidth = 6.f;
        sf::RectangleShape rect{ sf::Vector2f{ SquareWidth, SquareWidth } };
        rect.setFillColor(sf::Color::Blue);
        rect.setPosition(position.x - SquareWidth / 2.f, position.y - SquareWidth / 2.f);
        wpRects.push_back(rect);

        wpLines.append(sf::Vector2f{ (float) position.x, (float) position.y });

        controller.addWayPoint(display.pixelToVector(position.x, position.y));
    };

    // Add an initial waypoint in the centre of the window
    addWayPoint(sf::Vector2i{ display.WindowWidth / 2, display.WindowHeight / 2 });

    // For handling window events
    auto eventHandler = [&](const sf::Event &event) {
        // try driving the robot with the keyboard
        if (!isControllerOn && Viz::SFML::drive(robot, event)) {
            return;
        }

        // if user presses space, it toggles the controller on/off
        if (event.type == sf::Event::KeyReleased
            && event.key.code == sf::Keyboard::Space)
        {
            if (isControllerOn) {
                std::cout << "Controller off\n";

                robot.stopMoving();
                isControllerOn = false;
                lookAheadLine.clear();
            } else if (wpRects.size() > 1) {
                std::cout << "Controller on\n";

                controller.setLookAheadDistance(LookAheadDistance); // reset lookahead distance
                isControllerOn = true;
                warningDisplayed = false;
            } else {
                std::cerr << "Too few way points!\n";
            }

            return;
        }

        // the user can add waypoints by clicking
        if (event.type == sf::Event::MouseButtonPressed
            && event.mouseButton.button == sf::Mouse::Left)
        {
            addWayPoint(event.mouseButton);
        }
    };

    while (display.isOpen()) {
        car.setPose(robot.getPose());

        // update GUI; handle keyboard + mouse events
        display.drawAndHandleEvents(eventHandler, wpLines, wpRects, lookAheadLine, car);

        if (isControllerOn) {
            const auto lookPoint = controller.getLookAheadPoint(robot.getPose(), LookAheadDistance);

            // draw lookahead point and a line to it from the robot
            lookAheadLine.clear();
            if (lookPoint) {
                lookAheadLine.append(robot.getPose());
                lookAheadLine.append(lookPoint.value());
                warningDisplayed = false;
            } else if (!warningDisplayed) {
                LOGE << "Robot is stuck! 😭\n";
                warningDisplayed = true;
            }

            // calculate turning angle with controller
            const auto turningAngle = controller.getTurningAngle(robot.getPose(), lookPoint);
            if (turningAngle) {
                const auto ang = turningAngle.value();
                if (abs(ang) > MaxTurn) {
                    robot.move(MaxSpeed, copysign(MaxTurn, ang));
                } else {
                    robot.move(MaxSpeed, ang);
                }
            } else {
                robot.stopMoving();
            }
        }
    }

    return EXIT_SUCCESS;
}
