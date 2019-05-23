#pragma once

// BoB robotics includes
#include "common/pose.h"
#include "navigation/read_objects.h"
#include "robots/control/collision_detector.h"
#include "robots/control/obstacle_circumnavigation.h"
#include "robots/tank.h"
#include "viz/sfml_world/arena_object.h"
#include "viz/sfml_world/sfml_world.h"

// Eigen
#include <Eigen/Core>

// SFML
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::literals;
using namespace units::length;
using namespace std::literals;

template<class PoseGetterType>
class ObstacleCircumnavigationRunner
{
    using V = Vector2<meter_t>;

public:
    ObstacleCircumnavigationRunner(Robots::Tank &tank,
                                   PoseGetterType &poseGetter)
      : m_Tank(tank)
      , m_PoseGetter(poseGetter)
      , m_Display(V{ 5_m, 5_m })
      , m_CarAgent(m_Display.createCarAgent(tank.getRobotWidth()))
      , m_RouteLines(m_Display.createLineStrip(sf::Color::Blue))
    {
        // Read object vertices from file
        const auto objects = Navigation::readObjects("objects.yaml");

        // The x and y dimensions of the robot
        const auto halfWidth = m_CarAgent.getSize().x() / 2;
        const auto halfLength = m_CarAgent.getSize().y() / 2;
        const std::array<V, 4> robotDimensions = {
            V{ -halfWidth, halfLength },
            V{ halfWidth, halfLength },
            V{ halfWidth, -halfLength },
            V{ -halfWidth, -halfLength }
        };

        // Objects for controlling circumnavigation
        m_CollisionDetector = std::make_unique<Robots::CollisionDetector>(robotDimensions,
                                                                          objects,
                                                                          20_cm,
                                                                          1_cm);
        m_Circumnavigator = std::make_unique<Robots::ObstacleCircumnavigator<PoseGetterType>>(m_Tank,
                                                                                              m_PoseGetter,
                                                                                              *m_CollisionDetector);

        // Create drawable objects
        const auto &resizedObjects = m_CollisionDetector->getResizedObjects();
        m_ObjectShapes = Viz::ArenaObject::fromObjects(m_Display, objects, resizedObjects);
    }

    bool update()
    {
        // Run circumnavigator
        m_Circumnavigator->update();
        const auto &pose = m_PoseGetter.getPose();

        // If we've just started circumnavigation, draw the waypoints on screen
        if (m_Circumnavigator->getState() == Robots::ObstacleCircumnavigatorState::StartingCircumnavigation) {
            m_RouteLines.clear();

            // The robot's location is the first point
            m_RouteLines.append(pose);

            // Add all the waypoints to the linestrip
            for (auto &waypoint : m_Circumnavigator->getPIDWaypoints()) {
                m_RouteLines.append(waypoint);
            }
        }

        // Render display
        m_CarAgent.setPose(pose);
        if (m_Circumnavigator->getState() == Robots::ObstacleCircumnavigatorState::DoingNothing) {
            m_Display.updateAndDrive(m_Tank, m_ObjectShapes, m_CarAgent);
        } else {
            // Also draw routeLines
            m_Display.updateAndDrive(m_Tank, m_ObjectShapes, m_RouteLines, m_CarAgent);
        }

        // Signal whether or not to exit program
        return m_Display.isOpen();
    }

private:
    Robots::Tank &m_Tank;
    PoseGetterType &m_PoseGetter;
    Viz::SFMLWorld m_Display;
    Viz::SFMLWorld::CarAgent m_CarAgent;
    Viz::SFMLWorld::LineStrip m_RouteLines;
    std::vector<Viz::ArenaObject> m_ObjectShapes;
    std::unique_ptr<Robots::CollisionDetector> m_CollisionDetector;
    std::unique_ptr<Robots::ObstacleCircumnavigator<PoseGetterType>> m_Circumnavigator;
};

template<class PoseGetterType>
auto
createRunner(Robots::Tank &tank,
             PoseGetterType &poseGetter)
{
    return std::make_unique<ObstacleCircumnavigationRunner<PoseGetterType>>(tank, poseGetter);
}
