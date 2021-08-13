#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/pose.h"

// Third-party includes
#include "third_party/units.h"

// SFML
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <memory>
#include <vector>

namespace BoBRobotics {
namespace Viz {
namespace SFML {
using namespace units::literals;

/**!
 * \brief Drive a robot with an SFML window event
 *
 * Returns true if event was handled or false otherwise.
 */
template<class RobotType>
bool drive(RobotType &robot, const sf::Event &event)
{
    switch (event.type) {
    case sf::Event::KeyPressed:
        switch (event.key.code) {
        case sf::Keyboard::Up:
            robot.moveForward(1.f);
            return true;
        case sf::Keyboard::Down:
            robot.moveForward(-1.f);
            return true;
        case sf::Keyboard::Left:
            robot.turnOnTheSpot(-0.5f);
            return true;
        case sf::Keyboard::Right:
            robot.turnOnTheSpot(0.5f);
            return true;
        default:
            return false;
        }
        break;
    case sf::Event::KeyReleased:
        switch (event.key.code) {
        case sf::Keyboard::Up:
        case sf::Keyboard::Down:
        case sf::Keyboard::Left:
        case sf::Keyboard::Right:
            robot.stopMoving();
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

class CrossShape
  : public sf::Drawable
{
public:
    CrossShape(const sf::Vector2f &position, float size, float thickness, const sf::Color &colour);

    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const override;

private:
    sf::RectangleShape m_Horizontal, m_Vertical;
}; // CrossShape

class World
{
    using meter_t = units::length::meter_t;

public:
    class CarAgent
      : public sf::Drawable
    {
    public:
        CarAgent(const World &display, meter_t carWidth);

        const auto &getSize() const { return m_Size; }

        template<class PoseType>
        void setPose(const PoseType &pose)
        {
            m_Sprite.setRotation(-static_cast<units::angle::degree_t>(pose.yaw()).value());
            m_Sprite.setPosition(m_Display.vectorToPixel(pose));
        }

        virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const override;

    private:
        const World &m_Display;
        Vector2<meter_t> m_Size;
        sf::Texture m_Texture;
        sf::Sprite m_Sprite;
    };

    class LineStrip
      : public sf::Drawable
    {
    public:
        LineStrip(const World &renderer, const sf::Color &colour)
          : m_Renderer(renderer)
          , m_Colour(colour)
        {}

        virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const override
        {
            target.draw(&m_Vertices[0], m_Vertices.size(), sf::PrimitiveType::LinesStrip, states);
        }

        template<typename PositionType>
        void append(const PositionType &position)
        {
            append(m_Renderer.vectorToPixel(position));
        }

        void append(const sf::Vector2f &position)
        {
            m_Vertices.emplace_back(position, m_Colour);
        }

        void clear() { m_Vertices.clear(); }

    private:
        std::vector<sf::Vertex> m_Vertices;
        const World &m_Renderer;
        const sf::Color m_Colour;
    };

    static constexpr int WindowWidth = 800, WindowHeight = 800;

    World(const Vector2<meter_t> &arenaSize = { 3.2_m, 3.2_m });

    template<typename MaxBoundsType>
    World(const Vector2<meter_t> &minBounds,
              const MaxBoundsType &maxBounds)
      : m_Window(sf::VideoMode(WindowWidth, WindowHeight),
                 "BoB robotics",
                 sf::Style::Titlebar | sf::Style::Close,
                 getContextSettings())
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

        // Put cross at origin
        const auto origin = vectorToPixel(0.0, 0.0);
        m_OriginCross = std::make_unique<CrossShape>(origin, OriginLineLength, OriginLineThickness, sf::Color::Black);
    }

    CarAgent createCarAgent(meter_t carWidth = 16.4_cm);

    LineStrip createLineStrip(const sf::Color &colour) const
    {
        return LineStrip(*this, colour);
    }

    template<class VectorType>
    CrossShape createCrossShape(const VectorType &position, float size, float thickness, const sf::Color &colour)
    {
        return CrossShape(vectorToPixel(position), size, thickness, colour);
    }

    //! Update the GUI, drawing the specified drawables in the window
    template<typename... Drawables>
    void draw(Drawables&& ...drawables)
    {
        drawAndHandleEvents(Noop{}, std::forward<Drawables>(drawables)...);
    }

    //! As with draw(), except window events are also passed in sequence to eventHandler
    template<typename Func, typename... Drawables>
    void drawAndHandleEvents(Func eventHandler, Drawables&& ...drawables)
    {
        // Set m_Window to be active OpenGL context
        m_Window.setActive(true);

        // Check all the window's events that were triggered since the last iteration of the loop
        sf::Event event{};
        while (m_Window.pollEvent(event)) {
            if (handleWindowClosing(event)) {
                break;
            }

            eventHandler(event);
        }

        // Set background colour
        m_Window.clear(sf::Color::White);

        // Draw cross at origin and other things
        drawThings(*m_OriginCross, std::forward<Drawables>(drawables)...);

        // Swap buffers
        m_Window.display();

        // We don't need to be the current OpenGL context any more
        m_Window.setActive(false);
    }

    bool isOpen() const;
    void close();
    float lengthToPixel(const meter_t value) const;
    Vector2<meter_t> pixelToVector(int x, int y) const;
    sf::Window &getWindow();

    template<class VectorType>
    sf::Vector2f vectorToPixel(const VectorType &point) const
    {
        return { lengthToPixel(point.x() - m_MinBounds[0]),
                 static_cast<float>(WindowHeight) - lengthToPixel(point.y() - m_MinBounds[1]) };
    }

    sf::Vector2f vectorToPixel(double x, double y) const;

private:
    sf::RenderWindow m_Window;
    std::unique_ptr<CrossShape> m_OriginCross;
    const Vector2<meter_t> m_MinBounds;
    meter_t m_UnitPerPixel;

    static constexpr float OriginLineThickness = 3.f, OriginLineLength = 20.f;

    bool handleWindowClosing(sf::Event &event);

    template<typename DrawableType, typename... Drawables>
    void drawThings(const std::vector<DrawableType> &drawables, Drawables&& ...others)
    {
        for (const auto &drawable : drawables) {
            m_Window.draw(drawable);
        }
        drawThings(std::forward<Drawables>(others)...);
    }

    template<typename... Drawables>
    void drawThings(const sf::Drawable &drawable, Drawables&& ...others)
    {
        m_Window.draw(drawable);
        drawThings(std::forward<Drawables>(others)...);
    }

    void drawThings()
    {}

    struct Noop {
        void operator()(const sf::Event &)
        {}
    };

    static sf::ContextSettings getContextSettings();

}; // World

} // SFML
} // Viz
} // BobRobotics
