#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "common/pose.h"
#include "robots/robot.h"

// Third-party includes
#include "third_party/units.h"

// SFML
#include <SFML/Graphics.hpp>

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <vector>

namespace BoBRobotics {
namespace Viz {
using namespace units::literals;

class SFMLWorld
{
    using meter_t = units::length::meter_t;

public:
    class CarAgent
      : public sf::Drawable
    {
    public:
        CarAgent(const SFMLWorld &display, meter_t carWidth);

        template<class PoseType>
        void setPose(const PoseType &pose)
        {
            m_Sprite.setRotation(-static_cast<units::angle::degree_t>(pose.yaw()).value());
            m_Sprite.setPosition(m_Display.vectorToPixel(pose));
        }

        virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const override;

    private:
        const SFMLWorld &m_Display;
        sf::Texture m_Texture;
        sf::Sprite m_Sprite;
    };

    static constexpr int WindowWidth = 800, WindowHeight = 800;

    SFMLWorld(const Vector2<meter_t> &arenaSize = { 3.2_m, 3.2_m });

    template<typename MaxBoundsType>
    SFMLWorld(const Vector2<meter_t> &minBounds,
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

        const meter_t width = maxBounds[0] - minBounds[0];
        const meter_t height = maxBounds[1] - minBounds[1];
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

    CarAgent createCarAgent(meter_t carWidth = 16.4_cm);

    template<typename... Drawables>
    sf::Event update(Drawables&& ...drawables)
    {
        // Set m_Window to be active OpenGL context
        m_Window.setActive(true);

        // Check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        memset(static_cast<void *>(&event), 0, sizeof(event));
        while (m_Window.pollEvent(event)) {
            if (handleEvents(event)) {
                return event;
            }
        }

        // Draw on screen
        doDrawing(std::forward<Drawables>(drawables)...);

        // We don't need to be the current OpenGL context any more
        m_Window.setActive(false);

        return event;
    }

    template<typename... Drawables>
    sf::Event updateAndDrive(Robots::Robot &robot, Drawables&& ...drawables)
    {
        // Set m_Window to be active OpenGL context
        m_Window.setActive(true);

        // Check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        memset(static_cast<void *>(&event), 0, sizeof(event));
        while (m_Window.pollEvent(event)) {
            if (handleEvents(event)) {
                return event;
            }

            switch (event.type) {
            case sf::Event::KeyPressed:
                switch (event.key.code) {
                case sf::Keyboard::Up:
                    robot.moveForward(1.f);
                    break;
                case sf::Keyboard::Down:
                    robot.moveForward(-1.f);
                    break;
                case sf::Keyboard::Left:
                    robot.turnOnTheSpot(-0.5f);
                    break;
                case sf::Keyboard::Right:
                    robot.turnOnTheSpot(0.5f);
                    break;
                default:
                    break;
                }
                break;
            case sf::Event::KeyReleased:
                switch (event.key.code) {
                case sf::Keyboard::Up:
                case sf::Keyboard::Down:
                case sf::Keyboard::Left:
                case sf::Keyboard::Right:
                    robot.stopMoving();
                default:
                    break;
                }
            default:
                break;
            }
        }

        // Draw on screen
        doDrawing(std::forward<Drawables>(drawables)...);

        // We don't need to be the current OpenGL context any more
        m_Window.setActive(false);

        return event;
    }

    bool mouseClicked() const;
    auto mouseClickPosition() const;
    bool isOpen() const;
    void close();
    float lengthToPixel(const meter_t value) const;
    auto pixelToVector(int x, int y);

    template<class VectorType>
    sf::Vector2f vectorToPixel(const VectorType &point) const
    {
        return { lengthToPixel(point.x() - m_MinBounds[0]),
                 static_cast<float>(WindowHeight) - lengthToPixel(point.y() - m_MinBounds[1]) };
    }

    sf::Vector2f vectorToPixel(double x, double y) const;

private:
    sf::RenderWindow m_Window;
    sf::RectangleShape m_OriginLineHorizontal, m_OriginLineVertical;
    const Vector2<meter_t> m_MinBounds;
    meter_t m_UnitPerPixel;
    Vector2<meter_t> m_MouseClickPosition = Vector2<meter_t>::nan();

    static constexpr float OriginLineThickness = 3.f, OriginLineLength = 20.f;

    bool handleEvents(sf::Event &event);

    template<typename... Drawables>
    void doDrawing(Drawables&& ...drawables)
    {
        // Set background colour
        m_Window.clear(sf::Color::White);

        // Draw cross at origin
        m_Window.draw(m_OriginLineHorizontal);
        m_Window.draw(m_OriginLineVertical);

        // Draw extra drawable things
        draw(std::forward<Drawables>(drawables)...);

        // Swap buffers
        m_Window.display();
    }

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

    static sf::ContextSettings getContextSettings();

}; // SFMLWorld
} // Viz
} // BobRobotics
