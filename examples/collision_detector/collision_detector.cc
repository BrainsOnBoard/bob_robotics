// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/pose.h"
#include "hid/joystick.h"
#include "navigation/read_objects.h"
#include "net/client.h"
#include "vicon/udp.h"
#include "robots/control/collision_detector.h"
#include "robots/tank_netsink.h"
#include "viz/sfml_world/sfml_world.h"

// Eigen
#include <Eigen/Core>

// Standard C++ includes
#include <array>
#include <chrono>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace units::literals;
using namespace units::length;
using namespace std::literals;

class Agent
  : public sf::Drawable
{
public:
    Agent(const Viz::SFMLWorld &renderer, size_t numVertices)
      : m_Renderer(renderer)
      , m_Shape(numVertices)
    {
        m_Shape.setFillColor(sf::Color::Blue);
    }

    void setVertices(const Eigen::MatrixX2d &vertices)
    {
        for (int i = 0; i < vertices.rows(); i++) {
            m_Shape.setPoint(i, m_Renderer.vectorToPixel(vertices(i, 0), vertices(i, 1)));
        }
    }

    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const override
    {
        target.draw(m_Shape, states);
    }

private:
    const Viz::SFMLWorld &m_Renderer;
    sf::ConvexShape m_Shape;
};

class ArenaObject
  : public sf::Drawable
{
public:
    template<class VectorArrayType, class MatrixType>
    ArenaObject(const Viz::SFMLWorld &renderer, const VectorArrayType &original, const MatrixType &resized)
      : m_GreenShape(original.size())
      , m_RedShape(original.size())
    {
        // Dark green
        m_GreenShape.setFillColor(sf::Color{ 0x00, 0x88, 0x00 });

        // Add each vertex to the shape
        for (size_t i = 0; i < original.size(); i++) {
            m_GreenShape.setPoint(i, renderer.vectorToPixel(original[i]));
        }

        m_RedShape.setFillColor(sf::Color::Red);
        for (size_t i = 0; i < original.size(); i++) {
            m_RedShape.setPoint(i, renderer.vectorToPixel(resized(i, 0), resized(i, 1)));
        }
    }

    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const override
    {
        target.draw(m_RedShape, states);
        target.draw(m_GreenShape, states);
    }

private:
    sf::ConvexShape m_GreenShape, m_RedShape;
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
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    // Control robot with joystick over network
    HID::Joystick joystick;
    Net::Client client;
    Robots::TankNetSink tank(client);
    tank.addJoystick(joystick);

    // Display for robot + objects
    Viz::SFMLWorld renderer{ V{ 5_m, 5_m } };

    // Read objects from file
    const auto objects = Navigation::readObjects("objects.yaml");
    Robots::CollisionDetector collisionDetector{ robotDimensions, objects, 30_cm, 1_cm };

    // Object size + buffer around
    const auto &resizedObjects = collisionDetector.getResizedObjects();

    // Create drawable objects
    std::vector<ArenaObject> objectShapes;
    objectShapes.reserve(objects.size());
    for (size_t i = 0; i < objects.size(); i++) {
        objectShapes.emplace_back(renderer, objects[i], resizedObjects[i]);
    }

    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Ctrl+C etc.

    // Listen for messages on background thread
    client.runInBackground();

    bool printedCollisionMessage = false;
    Agent agent(renderer, robotDimensions.size()); // Drawable agent
    while (renderer.isOpen() && !joystick.isPressed(HID::JButton::B)) {
        // Check for exceptions on background thread
        catcher.check();

        // Poll for joystick events
        joystick.update();

        collisionDetector.setRobotPose(vicon.getObjectData(0).getPose());
        if (collisionDetector.collisionOccurred()) {
            if (!printedCollisionMessage) {
                tank.stopMoving();
                std::cout << "COLLISION!!!" << std::endl;
                printedCollisionMessage = true;
            }
        } else {
            printedCollisionMessage = false;
        }

        // Render on display
        agent.setVertices(collisionDetector.getRobotVertices());
        renderer.update(objectShapes, agent);

        // Small delay so we don't hog CPU
        std::this_thread::sleep_for(20ms);
    }
}
