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
public:
    static constexpr int WindowWidth = 800, WindowHeight = 600;

    AgentRenderer(const LengthUnit arenaHeight = 3.2_m)
      : m_Window(sf::VideoMode(WindowWidth, WindowHeight),
                 "BoB robotics",
                 sf::Style::Titlebar | sf::Style::Close)
      , m_OriginLineHorizontal({ OriginLineLength, OriginLineThickness })
      , m_OriginLineVertical({ OriginLineThickness, OriginLineLength })
      , m_UnitPerPixel(arenaHeight / WindowHeight)
    {
        m_Window.setVerticalSyncEnabled(true);

        // Put red cross at origin
        m_OriginLineHorizontal.setFillColor(sf::Color::Red);
        m_OriginLineVertical.setFillColor(sf::Color::Red);
        m_OriginLineHorizontal.setPosition({ (WindowWidth - OriginLineLength) / 2.f, (WindowHeight - OriginLineThickness) / 2.f });
        m_OriginLineVertical.setPosition({ (WindowWidth - OriginLineThickness) / 2.f, (WindowHeight - OriginLineLength) / 2.f });
    }

    void update()
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
                m_Objects.back().setPoint(i, { static_cast<float>((object[i][0] / m_UnitPerPixel).value()) + (WindowWidth / 2),
                                               -static_cast<float>((object[i][1] / m_UnitPerPixel).value()) + (WindowHeight / 2)});
            }
        }
    }

private:
    sf::RenderWindow m_Window;
    sf::RectangleShape m_OriginLineHorizontal, m_OriginLineVertical;
    std::vector<sf::ConvexShape> m_Objects;
    const LengthUnit m_UnitPerPixel;

    static constexpr float OriginLineThickness = 3.f, OriginLineLength = 20.f;
}; // AgentRenderer
} // Viz
} // BobRobotics
