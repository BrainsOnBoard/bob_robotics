#pragma once

// Standard C++ includes
#include <tuple>
#include <vector>

// OpenGL includes
#include <GL/glew.h>

// BoB robotics includes
#include "third_party/units.h"

//------------------------------------------------------------------------
// VectorField
//------------------------------------------------------------------------
class VectorField
{
public:
    VectorField(units::length::meter_t arrowLength)
    :   m_ArrowLength(arrowLength), m_NumX(0), m_NumY(0), m_LinesVAO(0), m_LinesPositionVBO(0)
    {
    }

    VectorField(units::length::meter_t arrowLength,
                units::length::meter_t startX, units::length::meter_t endX, units::length::meter_t gridX,
                units::length::meter_t startY, units::length::meter_t endY, units::length::meter_t gridY)
    :   VectorField(arrowLength)
    {
        createVertices(startX, endX, gridX,
                       startY, endY, gridY);
    }
    ~VectorField();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void createVertices(units::length::meter_t startX, units::length::meter_t endX, units::length::meter_t gridX,
                        units::length::meter_t startY, units::length::meter_t endY, units::length::meter_t gridY);

    // Renders the vector field
    void render();

    // Gets number of points used to render vector field
    unsigned int getNumPoints() const{ return m_NumX * m_NumY; }

    // Gets the coordinates of a specific point
    std::tuple<units::length::meter_t, units::length::meter_t> getPoint(unsigned int point) const;

    // Sets novelty of a given point
    void setNovelty(unsigned int point, const std::vector<float> &novelty);

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    units::length::meter_t m_StartX;
    units::length::meter_t m_StartY;
    units::length::meter_t m_GridX;
    units::length::meter_t m_GridY;

    const units::length::meter_t m_ArrowLength;

    unsigned int m_NumX;
    unsigned int m_NumY;

    GLuint m_LinesVAO;
    GLuint m_LinesPositionVBO;
};