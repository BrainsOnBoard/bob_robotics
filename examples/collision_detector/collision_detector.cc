// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/logging.h"
#include "common/pose.h"
#include "hid/joystick.h"
#include "navigation/read_objects.h"
#include "net/client.h"
#include "robots/control/collision_detector.h"
#include "robots/tank_netsink.h"
#include "vicon/udp.h"
#include "viz/sfml_world/arena_object.h"
#include "viz/sfml_world/sfml_world.h"

// Eigen
#include <Eigen/Core>

// Standard C++ includes
#include <array>
#include <chrono>
#include <thread>

using namespace BoBRobotics;
using namespace units::literals;
using namespace units::length;
using namespace std::literals;

class Agent
  : public sf::Drawable
{
public:
    Agent(const Viz::SFMLWorld &display, size_t numVertices)
      : m_Display(display)
      , m_Shape(numVertices)
    {
        m_Shape.setFillColor(sf::Color::Blue);
    }

    void setVertices(const Eigen::MatrixX2d &vertices)
    {
        for (int i = 0; i < vertices.rows(); i++) {
            m_Shape.setPoint(i, m_Display.vectorToPixel(vertices(i, 0), vertices(i, 1)));
        }
    }

    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const override
    {
        target.draw(m_Shape, states);
    }

private:
    const Viz::SFMLWorld &m_Display;
    sf::ConvexShape m_Shape;
};

int
main()
{
    // The x and y dimensions of the robot
    using V = Vector2<meter_t>;
    constexpr std::array<V, 4> robotDimensions = {
        V{ -75_mm, 70_mm},
        V{ 75_mm, 70_mm },
        V{ 75_mm, -120_mm },
        V{ -75_mm, -120_mm }
    };

    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);

    // Control robot with joystick over network
    HID::Joystick joystick;
    Net::Client client;
    Robots::TankNetSink tank(client);
    tank.addJoystick(joystick);

    // Display for robot + objects
    Viz::SFMLWorld display{ V{ 5_m, 5_m } };

    // Read objects from file
    const auto objects = Navigation::readObjects("objects.yaml");
    Robots::CollisionDetector collisionDetector{ robotDimensions, objects, 30_cm, 1_cm };

    // Object size + buffer around
    const auto &resizedObjects = collisionDetector.getResizedObjects();

    // Create drawable objects
    std::vector<Viz::ArenaObject> objectShapes;
    objectShapes.reserve(objects.size());
    for (size_t i = 0; i < objects.size(); i++) {
        objectShapes.emplace_back(display, objects[i], resizedObjects[i]);
    }

    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Ctrl+C etc.

    // Listen for messages on background thread
    client.runInBackground();

    bool printedCollisionMessage = false;
    Agent agent(display, robotDimensions.size()); // Drawable agent
    while (display.isOpen() && !joystick.isPressed(HID::JButton::B)) {
        // Check for exceptions on background thread
        catcher.check();

        // Poll for joystick events
        joystick.update();

        collisionDetector.setRobotPose(vicon.getObjectData().getPose());
        Vector2<meter_t> collisionPosition;
        if (collisionDetector.collisionOccurred(collisionPosition)) {
            if (!printedCollisionMessage) {
                tank.stopMoving();
                LOGI << "COLLISION occured at " << collisionPosition.x() << ", " << collisionPosition.y() << "!!!";
                printedCollisionMessage = true;
            }
        } else {
            printedCollisionMessage = false;
        }

        // Render on display
        agent.setVertices(collisionDetector.getRobotVertices());
        display.update(objectShapes, agent);
    }
}
