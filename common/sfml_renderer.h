#pragma once

// BoB robotics includes
#include "../common/assert.h"
#include "../common/pose.h"
#include "../robots/robot.h"

// Third-party includes
#include "../third_party/units.h"

// SFML
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <vector>
#include <utility>

namespace BoBRobotics {
using namespace units::literals;

template<typename LengthUnit = units::length::meter_t>
class SFMLRenderer
{
    static_assert(units::traits::is_length_unit<LengthUnit>::value, "LengthUnit is not a unit length type");

public:
    static constexpr int WindowWidth = 800, WindowHeight = 800;

    SFMLRenderer(const Vector2<LengthUnit> &arenaSize = { 3.2_m, 3.2_m })
      : SFMLRenderer(Vector2<LengthUnit>{ -arenaSize[0] / 2, -arenaSize[1] / 2 },
                     Vector2<LengthUnit>{ arenaSize[0] / 2, arenaSize[1] / 2 })
    {}

    template<typename MaxBoundsType>
    SFMLRenderer(const Vector2<LengthUnit> &minBounds,
                 const MaxBoundsType &maxBounds)
      : m_Window(sf::VideoMode(WindowWidth, WindowHeight),
                 "BoB robotics",
                 sf::Style::Titlebar | sf::Style::Close,
                 getContextSettings())
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
        m_OriginLineHorizontal.setFillColor(sf::Color::Black);
        m_OriginLineVertical.setFillColor(sf::Color::Black);
        const auto origin = vectorToPixel(0.0, 0.0);
        m_OriginLineHorizontal.setPosition({ origin.x - (OriginLineLength / 2.f),
                                             origin.y - (OriginLineThickness / 2.f) });
        m_OriginLineVertical.setPosition({ origin.x - (OriginLineThickness / 2.f),
                                           origin.y - (OriginLineLength / 2.f) });
    }

    template<typename... Drawables>
    void update(Drawables&& ...drawables)
    {
        // Set m_Window to be active OpenGL context
        m_Window.setActive(true);

        // Check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        while (m_Window.pollEvent(event)) {
            // "Close requested" event: we close the window
            if (event.type == sf::Event::Closed ||
                    (event.type == sf::Event::KeyReleased && event.key.code == sf::Keyboard::Q)) {
                m_Window.close();
                return;
            }
        }

        // Set background colour
        m_Window.clear(sf::Color::White);

        // Draw cross at origin
        m_Window.draw(m_OriginLineHorizontal);
        m_Window.draw(m_OriginLineVertical);

        // Draw extra drawable things
        draw(std::forward<Drawables>(drawables)...);

        // Swap buffers
        m_Window.display();

        // We don't need to be the current OpenGL context any more
        m_Window.setActive(false);
    }

    bool isOpen() const
    {
        return m_Window.isOpen();
    }

    void close()
    {
        m_Window.close();
    }

    float lengthToPixel(const LengthUnit value) const
    {
        return static_cast<float>((value / m_UnitPerPixel).value());
    }

    template<class VectorType>
    sf::Vector2f vectorToPixel(const VectorType &point) const
    {
        return { lengthToPixel(point.x() - m_MinBounds[0]),
                 static_cast<float>(WindowHeight) - lengthToPixel(point.y() - m_MinBounds[1]) };
    }

    sf::Vector2f vectorToPixel(double x, double y) const
    {
        return { lengthToPixel(LengthUnit{ x } - m_MinBounds[0]),
                 static_cast<float>(WindowHeight) - lengthToPixel(LengthUnit{ y } - m_MinBounds[1]) };
    }

private:
    sf::RenderWindow m_Window;
    sf::RectangleShape m_OriginLineHorizontal, m_OriginLineVertical;
    const Vector2<LengthUnit> m_MinBounds;
    LengthUnit m_UnitPerPixel;

    static constexpr float OriginLineThickness = 3.f, OriginLineLength = 20.f;

    template<typename DrawableType, typename... Drawables>
    void draw(const std::vector<DrawableType> &drawables, Drawables&& ...others)
    {
        for (auto &drawable : drawables) {
            m_Window.draw(drawable);
        }
        draw(std::forward<Drawables>(others)...);
    }

    template<typename... Drawables>
    void draw(const sf::Drawable &drawable, Drawables&& ...others)
    {
        m_Window.draw(drawable);
        draw(std::forward<Drawables>(others)...);
    }

    void draw()
    {}

    static sf::ContextSettings getContextSettings()
    {
        sf::ContextSettings settings;
        settings.antialiasingLevel = 8;
        return settings;
    }
}; // SFMLRenderer
} // BobRobotics
