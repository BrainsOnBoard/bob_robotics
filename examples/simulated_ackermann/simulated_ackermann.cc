// BoB robotics includes
#include "robots/ackermann/simulated_ackermann.h"
#include "viz/sfml/world.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::literals;
using namespace units::angle;
using namespace units::length;

int
bobMain(int, char **)
{
    constexpr auto MaxSpeed = 1.4_mps;
    constexpr auto MaxTurn = 30_deg;

    Robots::Ackermann::SimulatedAckermann robot(MaxSpeed, 500_mm); // simulated Ackermann car
    Viz::SFML::World display({ 10.2_m, 10.2_m });                  // For displaying the agent

    auto car = display.createCarAgent(160_mm);

    auto velocity = 0_mps;
    auto turn = 0_deg;
    while (display.isOpen()) {
        car.setPose(robot.getPose());

        auto eventHandler = [&](const sf::Event &event) {
            switch (event.type) {
            case sf::Event::KeyPressed:
                switch (event.key.code) {
                case sf::Keyboard::Up:
                    velocity = MaxSpeed;
                    break;
                case sf::Keyboard::Down:
                    velocity = -MaxSpeed;
                    break;
                case sf::Keyboard::Left:
                    turn = MaxTurn;
                    break;
                case sf::Keyboard::Right:
                    turn = -MaxTurn;
                default:
                    break;
                }
                break;
            case sf::Event::KeyReleased:
                switch (event.key.code) {
                case sf::Keyboard::Up:
                case sf::Keyboard::Down:
                    velocity = 0_mps;
                    break;
                case sf::Keyboard::Left:
                case sf::Keyboard::Right:
                    turn = 0_deg;
                default:
                    break;
                }
            default:
                break;
            }
        };

        // run GUI and handle events
        display.drawAndHandleEvents(eventHandler, car);

        // drive robot
        robot.move(velocity, turn);
    }

    return EXIT_SUCCESS;
}
