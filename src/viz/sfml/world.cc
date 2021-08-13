// BoB robotics includes
#include "viz/sfml/world.h"
#include "common/path.h"

// SFML
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <stdexcept>

using namespace units::literals;
using namespace units::length;

namespace BoBRobotics {
namespace Viz {
namespace SFML {

CrossShape::CrossShape(const sf::Vector2f &position, float size, float thickness, const sf::Color &colour)
  : m_Horizontal({ size, thickness })
  , m_Vertical({ thickness, size })
{
    m_Horizontal.setOrigin(size / 2.f, thickness / 2.f);
    m_Vertical.setOrigin(thickness / 2.f, size / 2.f);
    m_Horizontal.setPosition(position);
    m_Vertical.setPosition(position);
    m_Horizontal.setFillColor(colour);
    m_Vertical.setFillColor(colour);
}

void
CrossShape::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    target.draw(m_Horizontal, states);
    target.draw(m_Vertical, states);
}

constexpr float World::OriginLineThickness, World::OriginLineLength;

World::CarAgent::CarAgent(const World &display, meter_t carWidth)
  : m_Display(display)
{
    const auto imageFilePath = Path::getResourcesPath() / "car.bmp";
    if (!m_Texture.loadFromFile(imageFilePath.str())) {
        throw std::runtime_error("Could not load " + imageFilePath.str());
    }

    // Make car sprite: scale and set origin to centre of image
    const auto imageSize = m_Texture.getSize();
    m_Sprite.setTexture(m_Texture);
    const auto widthPx = display.lengthToPixel(carWidth);
    const auto scale = widthPx / static_cast<float>(imageSize.y);
    m_Sprite.setOrigin(imageSize.x / 2.f, imageSize.y / 2.f);
    m_Sprite.scale(scale, scale);
}

void
World::CarAgent::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    target.draw(m_Sprite, states);
}

World::World(const Vector2<meter_t> &arenaSize)
  : World(Vector2<meter_t>{ -arenaSize[0] / 2, -arenaSize[1] / 2 },
              Vector2<meter_t>{ arenaSize[0] / 2, arenaSize[1] / 2 })
{}

World::CarAgent
World::createCarAgent(meter_t carWidth)
{
    return CarAgent(*this, carWidth);
}

std::experimental::optional<sf::Vector2i>
World::mouseClickPosition() const
{
    return m_MouseClickPosition;
}

bool
World::isOpen() const
{
    return m_Window.isOpen();
}

void
World::close()
{
    m_Window.close();
}

float
World::lengthToPixel(const meter_t value) const
{
    return static_cast<float>((value / m_UnitPerPixel).value());
}

Vector2<meter_t>
World::pixelToVector(int x, int y) const
{
    return { m_MinBounds[0] + m_UnitPerPixel * x,
             m_MinBounds[1] + m_UnitPerPixel * (WindowHeight - y) };
}

sf::Vector2f
World::vectorToPixel(double x, double y) const
{
    return { lengthToPixel(meter_t{ x } - m_MinBounds[0]),
             static_cast<float>(WindowHeight) - lengthToPixel(meter_t{ y } - m_MinBounds[1]) };
}

bool
World::handleEvents(sf::Event &event)
{
    if (event.type == sf::Event::Closed ||
        (event.type == sf::Event::KeyReleased && event.key.code == sf::Keyboard::Escape)) {
        m_Window.close();
        return true;
    }

    // Left mouse button pressed
    if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
        m_MouseClickPosition = sf::Vector2i{ event.mouseButton.x, event.mouseButton.y };
    }
    return false;
}

sf::ContextSettings
World::getContextSettings()
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    return settings;
}

sf::Window &
World::getWindow()
{
    return m_Window;
}

} // SFML
} // Viz
} // BobRobotics
