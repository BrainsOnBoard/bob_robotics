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
    static constexpr int WindowWidth = 800, WindowHeight = 800;

    AgentRenderer(const LengthUnit agentSize = 30_cm,
                  const Vector2<LengthUnit> &arenaSize = { 3.2_m, 3.2_m })
      : AgentRenderer(agentSize,
                      { -arenaSize[0] / 2, -arenaSize[1] / 2 },
                      { arenaSize[0] / 2, arenaSize[1] / 2 })
    {}

    AgentRenderer(const LengthUnit agentSize,
                  const Vector2<LengthUnit> &minBounds,
                  const Vector2<LengthUnit> &maxBounds)
      : m_Window(sf::VideoMode(WindowWidth, WindowHeight),
                 "BoB robotics",
                 sf::Style::Titlebar | sf::Style::Close)
      , m_OriginLineHorizontal({ OriginLineLength, OriginLineThickness })
      , m_OriginLineVertical({ OriginLineThickness, OriginLineLength })
      , m_MinBounds(minBounds)
    {
        m_Window.setVerticalSyncEnabled(true);

        const LengthUnit width = maxBounds[0] - minBounds[0];
        const LengthUnit height = maxBounds[1] - minBounds[1];
        BOB_ASSERT(width > 0_m && height > 0_m);

        sf::Vector2u windowSize;
        if (width > height) {
            m_UnitPerPixel = width / WindowWidth;
            windowSize.x = WindowWidth;
            windowSize.y = static_cast<unsigned>((height / m_UnitPerPixel).value());
            m_Window.setSize(windowSize);
        } else {
            m_UnitPerPixel = height / WindowHeight;
            windowSize.x = static_cast<unsigned>((width / m_UnitPerPixel).value());
            windowSize.y = WindowHeight;
            m_Window.setSize(windowSize);
        }

        // Put red cross at origin
        m_OriginLineHorizontal.setFillColor(sf::Color::Red);
        m_OriginLineVertical.setFillColor(sf::Color::Red);
        const auto origin = lengthToVector(0_m, 0_m);
        m_OriginLineHorizontal.setPosition({ origin.x - (OriginLineLength / 2.f),
                                             origin.y - (OriginLineThickness / 2.f) });
        m_OriginLineVertical.setPosition({ origin.x - (OriginLineThickness / 2.f),
                                           origin.y - (OriginLineLength / 2.f) });

        // Set size of agent marker
        m_Agent.setWidth(lengthToPixel(agentSize));
    }

    void update(const Pose2<LengthUnit, units::angle::degree_t> &agentPose)
    {
        // Set m_Window to be active OpenGL context
        m_Window.setActive(true);

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
        m_Agent.draw(m_Window,
                     lengthToVector(agentPose.x, agentPose.y),
                     -agentPose.angle);

        // Swap buffers
        m_Window.display();

        // We don't need to be the current OpenGL context any more
        m_Window.setActive(false);
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
    class AgentMarker
    {
    public:
        AgentMarker()
        {
            m_Square.setFillColor(sf::Color::Transparent);
            m_Square.setOutlineColor(sf::Color::Black);
            m_Square.setOutlineThickness(2.f);

            m_Line[0].color = m_Line[1].color = sf::Color::Black;
        }

        void draw(sf::RenderWindow &window, const sf::Vector2f position, const units::angle::degree_t rotation)
        {
            // Set square's position
            const float width = m_Square.getSize().x;
            const float halfWidth = width / 2.f;
            m_Square.setPosition({ position.x - halfWidth, position.y - halfWidth });

            // Make transform for rotating square
            sf::Transform transform;
            transform.rotate(static_cast<float>(rotation.value()), position);

            // Draw square
            window.draw(m_Square, transform);

            // Draw line
            using namespace units::math;
            m_Line[0].position = position;
            m_Line[1].position = { position.x + width * static_cast<float>(cos(rotation).value()),
                                   position.y + width * static_cast<float>(sin(rotation).value()) };
            window.draw(&m_Line[0], 2u, sf::PrimitiveType::Lines);
        }

        void setWidth(const float width)
        {
            m_Square.setSize({ width, width });
        }

    private:
        sf::RectangleShape m_Square;
        sf::Vertex m_Line[2];
    };

    sf::RenderWindow m_Window;
    sf::RectangleShape m_OriginLineHorizontal, m_OriginLineVertical;
    AgentMarker m_Agent;
    std::vector<sf::ConvexShape> m_Objects;
    const Vector2<LengthUnit> m_MinBounds;
    LengthUnit m_UnitPerPixel;

    static constexpr float OriginLineThickness = 3.f, OriginLineLength = 20.f;

    float lengthToPixel(const LengthUnit value) const
    {
        return static_cast<float>((value / m_UnitPerPixel).value());
    }

    sf::Vector2f lengthToVector(const LengthUnit x, const LengthUnit y) const
    {
        return { lengthToPixel(x - m_MinBounds[0]),
                 static_cast<float>(WindowHeight) - lengthToPixel(y - m_MinBounds[1]) };
    }
}; // AgentRenderer
} // Viz
} // BobRobotics
