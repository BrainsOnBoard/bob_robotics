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
    using degree_t = units::angle::degree_t;
    using meter_t = units::length::meter_t;
public:
    VectorField(meter_t arrowLength)
    :   m_ArrowLength(arrowLength), m_NumX(0), m_NumY(0), m_LinesVAO(0), m_LinesPositionVBO(0)
    {
    }

    VectorField(meter_t arrowLength,
                meter_t startX, meter_t endX, meter_t gridX,
                meter_t startY, meter_t endY, meter_t gridY)
    :   VectorField(arrowLength)
    {
        createVertices(startX, endX, gridX,
                       startY, endY, gridY);
    }
    ~VectorField();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void createVertices(meter_t startX, meter_t endX, meter_t gridX,
                        meter_t startY, meter_t endY, meter_t gridY);

    // Renders the vector field
    void render(meter_t height = meter_t{0.1});

    // Gets number of points used to render vector field
    unsigned int getNumPoints() const{ return m_NumX * m_NumY; }

    // Gets the coordinates of a specific point
    std::tuple<meter_t, meter_t> getPoint(unsigned int point) const;

    // Sets novelty of a given point
    void setNovelty(unsigned int point, const std::vector<std::pair<degree_t, float>> &novelty);

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    meter_t m_StartX;
    meter_t m_StartY;
    meter_t m_GridX;
    meter_t m_GridY;

    const meter_t m_ArrowLength;

    unsigned int m_NumX;
    unsigned int m_NumY;

    GLuint m_LinesVAO;
    GLuint m_LinesPositionVBO;
};
