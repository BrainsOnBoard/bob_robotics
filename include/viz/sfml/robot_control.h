#pragma once

// BoB robotics includes
#include "robots/type_traits.h"

// SFML
#include <SFML/Graphics.hpp>

namespace BoBRobotics {
namespace Viz {
namespace SFML {

/**!
 * \brief Drive a robot with an SFML window event
 *
 * Returns true if event was handled or false otherwise.
 */
template<class RobotType,
         std::enable_if_t<!Robots::IsAckermann<RobotType>::value, int> = 0>
bool drive(RobotType &robot, const sf::Event &event)
{
    switch (event.type) {
    case sf::Event::KeyPressed:
        switch (event.key.code) {
        case sf::Keyboard::Up:
            robot.moveForward(1.f);
            return true;
        case sf::Keyboard::Down:
            robot.moveForward(-1.f);
            return true;
        case sf::Keyboard::Left:
            robot.turnOnTheSpot(-0.5f);
            return true;
        case sf::Keyboard::Right:
            robot.turnOnTheSpot(0.5f);
            return true;
        default:
            return false;
        }
        break;
    case sf::Event::KeyReleased:
        switch (event.key.code) {
        case sf::Keyboard::Up:
        case sf::Keyboard::Down:
        case sf::Keyboard::Left:
        case sf::Keyboard::Right:
            robot.stopMoving();
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

/**!
 * \brief Drive an Ackermann-type robot with an SFML window event
 *
 * The types of maxSpeed and maxTurn are templatable so that one can pass in a
 * speed in e.g. m/s. Defaults to using maximum speed and turn.
 *
 * Returns true if event was handled or false otherwise.
 */
template<class RobotType, class SpeedType = float, class TurnType = float,
         std::enable_if_t<Robots::IsAckermann<RobotType>::value, int> = 0>
bool
drive(RobotType &robot, const sf::Event &event, SpeedType maxSpeed = 1.f,
      TurnType maxTurn = 1.f)
{
    /*
     * We need to keep track of these values between calls. This is a slightly
     * gross way of doing things, but making a templatable RobotControl class
     * to manage state instead seems grosser.
     */
    static SpeedType velocity{};
    static TurnType turn{};

    bool handled = true;
    switch (event.type) {
    case sf::Event::KeyPressed:
        switch (event.key.code) {
        case sf::Keyboard::Up:
            velocity = maxSpeed;
            break;
        case sf::Keyboard::Down:
            velocity = -maxSpeed;
            break;
        case sf::Keyboard::Left:
            turn = maxTurn;
            break;
        case sf::Keyboard::Right:
            turn = -maxTurn;
            break;
        default:
            handled = false;
            break;
        }
        break;
    case sf::Event::KeyReleased:
        switch (event.key.code) {
        case sf::Keyboard::Up:
        case sf::Keyboard::Down:
            velocity = SpeedType{ 0 };
            break;
        case sf::Keyboard::Left:
        case sf::Keyboard::Right:
            turn = TurnType{ 0 };
            break;
        default:
            handled = false;
            break;
        }
        break;
    default:
        handled = false;
        break;
    }

    // Drive the robot
    if (handled) {
        robot.move(velocity, turn);
    }

    return handled;
}

} // SFML
} // Viz
} // BoBRobotics
