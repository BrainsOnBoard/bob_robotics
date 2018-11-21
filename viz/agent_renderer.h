#pragma once

// SFML
#include <SFML/Graphics.hpp>

namespace BoBRobotics {
namespace Viz {
class AgentRenderer
{
public:
    static constexpr int WindowWidth = 800, WindowHeight = 600;

    AgentRenderer()
      : m_Window(sf::VideoMode(WindowWidth, WindowHeight),
                 "BoB robotics",
                 sf::Style::Titlebar | sf::Style::Close)
      , m_OriginLineHorizontal({ OriginLineLength, OriginLineThickness })
      , m_OriginLineVertical({ OriginLineThickness, OriginLineLength })
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

        // Swap buffers
        m_Window.display();
    }

    bool isOpen() const
    {
        return m_Window.isOpen();
    }

private:
    sf::RenderWindow m_Window;
    sf::RectangleShape m_OriginLineHorizontal, m_OriginLineVertical;

    static constexpr float OriginLineThickness = 3.f, OriginLineLength = 20.f;
}; // AgentRenderer

#ifndef NO_HEADER_DEFINITIONS
constexpr int AgentRenderer::WindowWidth, AgentRenderer::WindowHeight;
constexpr float AgentRenderer::OriginLineThickness, AgentRenderer::OriginLineLength;
#endif
} // Viz
} // BobRobotics
