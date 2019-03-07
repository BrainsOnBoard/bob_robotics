#pragma once

// BoB robotics includes
#include "common/collision.h"
#include "common/obstacle_circumnavigation.h"
#include "common/pose.h"
#include "common/read_objects.h"
#include "common/sfml_display.h"
#include "robots/tank.h"

// Eigen
#include <Eigen/Core>

// SFML
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::literals;
using namespace units::length;
using namespace std::literals;

class ArenaObject
 : public sf::Drawable
{
public:
    template<class VectorArrayType, class MatrixType>
    ArenaObject(const SFMLDisplay<> &display, const VectorArrayType &original, const MatrixType &resized)
        : m_GreenShape(original.size())
        , m_RedShape(original.size())
    {
        // Dark green
        m_GreenShape.setFillColor(sf::Color{ 0x00, 0x88, 0x00 });

        // Add each vertex to the shape
        for (size_t i = 0; i < original.size(); i++) {
            m_GreenShape.setPoint(i, display.vectorToPixel(original[i]));
        }

        m_RedShape.setFillColor(sf::Color::Red);
        for (size_t i = 0; i < original.size(); i++) {
            m_RedShape.setPoint(i, display.vectorToPixel(resized(i, 0), resized(i, 1)));
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

struct Noop
{
    void operator()()
    {}
};

template<class PoseGetterType, class Func = Noop>
void
runObstacleCircumnavigation(Robots::Tank &tank, PoseGetterType &poseGetter, Func extraCalls = Noop())
{
    using V = Vector2<meter_t>;

    // Display for robot + objects
    SFMLDisplay<> display{ V{ 5_m, 5_m } };
    auto car = display.createCarAgent(tank.getRobotWidth());
    const auto halfWidth = car.getSize().x() / 2;
    const auto halfLength = car.getSize().y() / 2;

    // The x and y dimensions of the robot
    const std::array<V, 4> robotDimensions = {
        V{ -halfWidth, halfLength },
        V{ halfWidth, halfLength },
        V{ halfWidth, -halfLength },
        V{ -halfWidth, -halfLength }
    };

    // Read object vertices from file
    const auto objects = readObjects("objects.yaml");

    // Objects for controlling circumnavigation
    CollisionDetector collisionDetector{ robotDimensions, objects, 20_cm, 1_cm };
    ObstacleCircumnavigator<PoseGetterType> circum(tank, poseGetter, collisionDetector);

    // Create drawable objects
    const auto &resizedObjects = collisionDetector.getResizedObjects();
    std::vector<ArenaObject> objectShapes;
    objectShapes.reserve(objects.size());
    for (size_t i = 0; i < objects.size(); i++) {
        objectShapes.emplace_back(display, objects[i], resizedObjects[i]);
    }

    // For drawing the agent's route around the perimeter
    auto routeLines = display.createLine(sf::Color::Blue);

    do {
        // Extra functions specified by caller
        extraCalls();

        // Run circumnavigator
        circum.update();
        const auto &pose = poseGetter.getPose();

        // If we've just started circumnavigation, draw the waypoints on screen
        if (circum.getState() == ObstacleCircumnavigatorState::StartingCircumnavigation) {
            routeLines.clear();

            // The robot's location is the first point
            routeLines.append(pose);

            // Add all the waypoints to the linestrip
            for (auto &waypoint : circum.getPIDWaypoints()) {
                routeLines.append(waypoint);
            }
        }

        // Render display
        car.setPose(pose);
        if (circum.getState() == ObstacleCircumnavigatorState::DoingNothing) {
            display.updateAndDrive(tank, objectShapes, car);
        } else {
            // Also draw routeLines
            display.updateAndDrive(tank, objectShapes, routeLines, car);
        }

        // Small delay so we don't hog CPU
        std::this_thread::sleep_for(20ms);
    } while (display.isOpen());
}
