#pragma once

// BoB robotics includes
#include "../common/assert.h"
#include "../common/pose.h"

// Third-party includes
#include "../third_party/units.h"

// SFML
#include <SFML/Graphics.hpp>

namespace BoBRobotics {
namespace Viz {
using namespace units::literals;

template<typename LengthUnit = units::length::millimeter_t>
class AgentRenderer
{
    static_assert(units::traits::is_length_unit<LengthUnit>::value, "LengthUnit is not a unit length type");

public:
    static constexpr int WindowWidth = 800, WindowHeight = 600;

    AgentRenderer(const LengthUnit agentSize, const LengthUnit arenaHeight = 3.2_m)
      : m_Window(sf::VideoMode(WindowWidth, WindowHeight),
                 "BoB robotics",
                 sf::Style::Titlebar | sf::Style::Close)
      , m_OriginLineHorizontal({ OriginLineLength, OriginLineThickness })
      , m_OriginLineVertical({ OriginLineThickness, OriginLineLength })
      , m_Agent(50.f)
      , m_UnitPerPixel(arenaHeight / WindowHeight)
    {
        m_Window.setVerticalSyncEnabled(true);

        // Put red cross at origin
        m_OriginLineHorizontal.setFillColor(sf::Color::Red);
        m_OriginLineVertical.setFillColor(sf::Color::Red);
        m_OriginLineHorizontal.setPosition({ (WindowWidth - OriginLineLength) / 2.f, (WindowHeight - OriginLineThickness) / 2.f });
        m_OriginLineVertical.setPosition({ (WindowWidth - OriginLineThickness) / 2.f, (WindowHeight - OriginLineLength) / 2.f });

        // Set agent's colour
        m_Agent.setRadius(lengthToPixel(agentSize));
        m_Agent.setFillColor(sf::Color::Blue);
    }

    void update(const Pose2<LengthUnit, units::angle::degree_t> &agentPose)
    {
        // Check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        while (m_Window.pollEvent(event)) {
            // "Close requested" event: we close the window
            if (event.type == sf::Event::Closed) {
                m_Window.close();
            }
        }

        // Set background colour
        m_Window.clear(sf::Color::White);

        // Draw cross at origin
        m_Window.draw(m_OriginLineHorizontal);
        m_Window.draw(m_OriginLineVertical);

        // Draw objects
        for (auto &object : m_Objects) {
            m_Window.draw(object);
        }

        // Draw agent
        auto point = lengthToVector(agentPose.x, agentPose.y);
        point.x -= m_Agent.getRadius();
        point.y -= m_Agent.getRadius();
        m_Agent.setPosition(point);
        m_Window.draw(m_Agent);

        // Swap buffers
        m_Window.display();
    }

    bool isOpen() const
    {
        return m_Window.isOpen();
    }

    void addObjects(const std::vector<std::vector<Vector2<LengthUnit>>> &objects)
    {
        BOB_ASSERT(m_Objects.empty());
        m_Objects.reserve(objects.size());

        for (auto &object : objects) {
            m_Objects.emplace_back();
            m_Objects.back().setPointCount(object.size());
            m_Objects.back().setFillColor(sf::Color::Black);
            for (size_t i = 0; i < object.size(); i++) {
                m_Objects.back().setPoint(i, lengthToVector(object[i][0], object[i][1]));
            }
        }
    }

private:
    sf::RenderWindow m_Window;
    sf::RectangleShape m_OriginLineHorizontal, m_OriginLineVertical;
    sf::CircleShape m_Agent;
    std::vector<sf::ConvexShape> m_Objects;
    const LengthUnit m_UnitPerPixel;

    static constexpr float OriginLineThickness = 3.f, OriginLineLength = 20.f;

    float lengthToPixel(const LengthUnit value) const
    {
        return static_cast<float>((value / m_UnitPerPixel).value());
    }

    sf::Vector2f lengthToVector(const LengthUnit x, const LengthUnit y) const
    {
        return { lengthToPixel(x) + (WindowWidth / 2),
                 -lengthToPixel(y) + (WindowHeight / 2) };
    }
}; // AgentRenderer
} // Viz
} // BobRobotics
