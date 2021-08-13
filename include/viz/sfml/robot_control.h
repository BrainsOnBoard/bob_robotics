#pragma once

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
template<class RobotType>
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
} // SFML
} // Viz
} // BoBRobotics
