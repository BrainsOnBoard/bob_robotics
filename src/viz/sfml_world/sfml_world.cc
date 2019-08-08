// BoB robotics includes
#include "viz/sfml_world/sfml_world.h"

// SFML
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <stdexcept>

using namespace units::literals;
using namespace units::length;

namespace BoBRobotics {
namespace Viz {

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

void CrossShape::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    target.draw(m_Horizontal, states);
    target.draw(m_Vertical, states);
}

constexpr float SFMLWorld::OriginLineThickness, SFMLWorld::OriginLineLength;

SFMLWorld::CarAgent::CarAgent(const SFMLWorld &display, meter_t carWidth)
  : m_Display(display)
{
    const char *brPath = std::getenv("BOB_ROBOTICS_PATH");
    if (!brPath) {
        throw std::runtime_error("BOB_ROBOTICS_PATH environment variable is not set");
    }

    const std::string imageFilePath = std::string(brPath) + "/resources/car.bmp";
    if (!m_Texture.loadFromFile(imageFilePath)) {
        throw std::runtime_error("Could not load " + imageFilePath);
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
SFMLWorld::CarAgent::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    target.draw(m_Sprite, states);
}

SFMLWorld::SFMLWorld(const Vector2<meter_t> &arenaSize)
    : SFMLWorld(Vector2<meter_t>{ -arenaSize[0] / 2, -arenaSize[1] / 2 },
                    Vector2<meter_t>{ arenaSize[0] / 2, arenaSize[1] / 2 })
{}

SFMLWorld::CarAgent SFMLWorld::createCarAgent(meter_t carWidth)
{
    return CarAgent(*this, carWidth);
}

bool SFMLWorld::mouseClicked() const
{
    return !m_MouseClickPosition.isnan();
}

Vector2<meter_t> SFMLWorld::mouseClickPosition() const
{
    return m_MouseClickPosition;
}

bool SFMLWorld::isOpen() const
{
    return m_Window.isOpen();
}

void SFMLWorld::close()
{
    m_Window.close();
}

float SFMLWorld::lengthToPixel(const meter_t value) const
{
    return static_cast<float>((value / m_UnitPerPixel).value());
}

Vector2<meter_t> SFMLWorld::pixelToVector(int x, int y)
{
    return Vector2<meter_t>(m_MinBounds[0] + m_UnitPerPixel * x,
                                m_MinBounds[1] + m_UnitPerPixel * (WindowHeight - y));
}

sf::Vector2f SFMLWorld::vectorToPixel(double x, double y) const
{
    return { lengthToPixel(meter_t{ x } - m_MinBounds[0]),
                static_cast<float>(WindowHeight) - lengthToPixel(meter_t{ y } - m_MinBounds[1]) };
}

bool SFMLWorld::handleEvents(sf::Event &event)
{
    m_MouseClickPosition = Vector2<meter_t>::nan();

    if (event.type == sf::Event::Closed ||
            (event.type == sf::Event::KeyReleased && event.key.code == sf::Keyboard::Q)) {
        m_Window.close();
        return true;
    }

    // Left mouse button pressed
    if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
        m_MouseClickPosition = pixelToVector(event.mouseButton.x, event.mouseButton.y);
    }
    return false;
}

sf::ContextSettings SFMLWorld::getContextSettings()
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    return settings;
}

} // Viz
} // BobRobotics
