#include "vector_field.h"

// Standard C++ includes
#include <algorithm>

// Standard C includes
#include <cmath>
#include <cstdlib>

// Libantworld includes
#include "antworld/common.h"

using namespace units::literals;
using namespace units::angle;
using namespace units::length;

//------------------------------------------------------------------------
// VectorField
//------------------------------------------------------------------------
void VectorField::createVertices(meter_t startX, meter_t endX, meter_t gridX,
                                 meter_t startY, meter_t endY, meter_t gridY)
{
    // Cache grid dimensions
    m_StartX = startX;
    m_StartY = startY;
    m_GridX = gridX;
    m_GridY = gridY;
    m_NumX = (unsigned int)units::math::ceil((endX - startX) / gridX);
    m_NumY = (unsigned int)units::math::ceil((endY - startY) / gridY);

    // Create a vertex array object to bind everything together
    glGenVertexArrays(1, &m_LinesVAO);

    // Generate vertex buffer objects for positions
    glGenBuffers(1, &m_LinesPositionVBO);

    // Bind vertex array
    glBindVertexArray(m_LinesVAO);

    {
        // Create vector to hold 2D start and end positions for a line from each point
        std::vector<float> positions;
        positions.reserve(getNumPoints() * (2 * 2));

        // Loop through points
        for(unsigned int i = 0; i < getNumPoints(); i++) {
            // Get position of point in grid
            meter_t x;
            meter_t y;
            std::tie(x, y) = getPoint(i);

            // Initially set start and end of line to this position
            positions.push_back((float)x.value());
            positions.push_back((float)y.value());
            positions.push_back((float)x.value());
            positions.push_back((float)y.value());
        }

        glBindBuffer(GL_ARRAY_BUFFER, m_LinesPositionVBO);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(GLfloat), positions.data(), GL_DYNAMIC_DRAW);

        // Set vertex pointer and enable client state in VAO
        glVertexPointer(2, GL_FLOAT, 0, BUFFER_OFFSET(0));
        glEnableClientState(GL_VERTEX_ARRAY);
    }
}
//------------------------------------------------------------------------
VectorField::~VectorField()
{
    glDeleteBuffers(1, &m_LinesPositionVBO);
    glDeleteVertexArrays(1, &m_LinesVAO);
}
//------------------------------------------------------------------------
void VectorField::render()
{
    // Bind lines VAO
    glBindVertexArray(m_LinesVAO);

    // Draw lines
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 0.1f);
    glDrawArrays(GL_LINES, 0, getNumPoints() * 2);
    glPopMatrix();
}
//------------------------------------------------------------------------
std::tuple<meter_t, meter_t> VectorField::getPoint(unsigned int point) const
{
    // Convert index into x and y grid coordinates
    const auto index = div(point, m_NumX);
    const unsigned int xGrid = index.rem;
    const unsigned int yGrid = index.quot;

    return std::make_tuple(m_StartX + ((float)xGrid * m_GridX),
                           m_StartY + ((float)yGrid * m_GridY));
}
//------------------------------------------------------------------------
void VectorField::setNovelty(unsigned int point, const std::vector<std::pair<units::angle::degree_t, float>> &novelty)
{
    // Find most familiar sample
    const auto mostFamiliar = std::min_element(novelty.cbegin(), novelty.cend(),
        [](const auto &a, const auto &b){ return a.second < b.second; });

    // Get position of start of this point's vector
    meter_t x;
    meter_t y;
    std::tie(x, y) = getPoint(point);

    // Add arrow length in most familiar direction
    x += m_ArrowLength * units::math::sin(mostFamiliar->first).value();
    y += m_ArrowLength * units::math::cos(mostFamiliar->first).value();

    // Update end of this point's field line in buffer
    float position[2]{ (float)x.value(), (float)y.value() };
    glBindBuffer(GL_ARRAY_BUFFER, m_LinesPositionVBO);
    glBufferSubData(GL_ARRAY_BUFFER, (point * sizeof(float) * 4),
                    sizeof(float) * 2, position);

}