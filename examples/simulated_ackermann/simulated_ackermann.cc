// BoB robotics includes
#include "robots/ackermann/simulated_ackermann.h"
#include "viz/sfml/robot_control.h"
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

    Robots::Ackermann::SimulatedAckermann robot(MaxSpeed, 500_mm, 0_m, MaxTurn);
    Viz::SFML::World display({ 10.2_m, 10.2_m });

    auto car = display.createCarAgent(160_mm);

    while (display.isOpen()) {
        car.setPose(robot.getPose());

        auto eventHandler = [&](const sf::Event &event) {
            Viz::SFML::drive(robot, event);
        };

        // run GUI and handle events
        display.drawAndHandleEvents(eventHandler, car);
    }

    return EXIT_SUCCESS;
}
