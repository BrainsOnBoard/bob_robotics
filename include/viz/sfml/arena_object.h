#pragma once

// BoB robotics includes
#include "sfml_world.h"

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics {
namespace Viz {
class ArenaObject
  : public sf::Drawable
{
public:
    template<class VectorArrayType, class MatrixType>
    ArenaObject(const SFMLWorld &display, const VectorArrayType &original, const MatrixType &resized)
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

    template<class VectorOfObjects, class VectorOfMatrices>
    static auto fromObjects(const SFMLWorld &display,
                            const VectorOfObjects &objects,
                            const VectorOfMatrices &resized)
    {
        std::vector<ArenaObject> shapes;
        shapes.reserve(objects.size());
        for (size_t i = 0; i < objects.size(); i++) {
            shapes.emplace_back(display, objects[i], resized[i]);
        }
        return shapes;
    }

private:
    sf::ConvexShape m_GreenShape, m_RedShape;
}; // ArenaObject
} // Viz
} // BoBRobotics
