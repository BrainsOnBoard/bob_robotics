// BoB robotics includes
#include "common/macros.h"

// BoB robotics includes
#include "antworld/common.h"
#include "antworld/render_mesh.h"

// Standard C++ includes
#include <vector>

using namespace units::angle;
using namespace units::math; // cmath functions for unit types

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMesh
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
void RenderMesh::render() const
{
    // Bind surface
    m_Surface.bind();

    // Render surface
    m_Surface.render();

    // Unbind surface
    m_Surface.unbind();
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMeshSpherical
//----------------------------------------------------------------------------
RenderMeshSpherical::RenderMeshSpherical(degree_t horizontalFOV, degree_t verticalFOV, degree_t startLongitude,
                                         unsigned int numHorizontalSegments, unsigned int numVerticalSegments)
{
    // We need a vertical for each segment and one extra
    const unsigned int numHorizontalVerts = numHorizontalSegments + 1;
    const unsigned int numVerticalVerts = numVerticalSegments + 1;

    // Bind surface
    getSurface().bind();

    {
        // Calculate number of vertices in mesh
        const unsigned int numVertices = numHorizontalVerts * numVerticalVerts;

        // Reserve 2 XY positions and 2 SRT texture coordinates for each vertical
        std::vector<GLfloat> positions;
        std::vector<GLfloat> textureCoords;
        positions.reserve(numVertices * 2);
        textureCoords.reserve(numVertices * 3);

        const float segmentWidth = 1.0f / (float)numHorizontalSegments;
        const degree_t startLatitude = -horizontalFOV / 2.0;
        const degree_t latitudeStep = horizontalFOV / numHorizontalSegments;

        const float segmentHeight = 1.0f / (float)numVerticalSegments;
        const degree_t longitudeStep = -verticalFOV / numVerticalSegments;

        // Loop through vertices
        for(unsigned int j = 0; j < numVerticalVerts; j++) {
            // Calculate screenspace segment y position
            const float y = segmentHeight * (float)j;

            // Calculate angle of horizontal and calculate its sin and cos
            const degree_t longitude = startLongitude + longitudeStep * j;
            const GLfloat sinLongitude = sin(longitude);
            const GLfloat cosLongitude = cos(longitude);

            for(unsigned int i = 0; i < numHorizontalVerts; i++) {
                // Calculate screenspace segment x position
                const float x = segmentWidth * (float)i;

                // Calculate angle of vertical and calculate it's sin and cos
                const degree_t latitude = startLatitude + latitudeStep * i;
                const GLfloat sinLatitude = sin(latitude);
                const GLfloat cosLatitude = cos(latitude);

                // Add vertex position
                positions.push_back(x);
                positions.push_back(y);

                // Add vertex texture coordinate
                textureCoords.push_back(sinLatitude * cosLongitude);
                textureCoords.push_back(sinLongitude);
                textureCoords.push_back(cosLatitude * cosLongitude);
            }
        }

        // Upload positions and texture coordinates
        getSurface().uploadPositions(positions, 2);
        getSurface().uploadTexCoords(textureCoords, 3);
    }

    {
        // Reserce indices for quads required to draw mesh
        std::vector<GLuint> indices;
        indices.reserve(numHorizontalSegments * numVerticalSegments * 4);

        // Loop through quads
        for(unsigned int y = 0; y < numVerticalSegments; y++) {
            for(unsigned int x = 0; x < numHorizontalSegments; x++) {
                indices.push_back((y * numHorizontalVerts) + x);
                indices.push_back((y * numHorizontalVerts) + x + 1);
                indices.push_back(((y + 1) * numHorizontalVerts) + x + 1);
                indices.push_back(((y + 1) * numHorizontalVerts) + x);
            }
        }

        // Upload indices to surface
        getSurface().uploadIndices(indices, GL_QUADS);
    }

    // Unbind surface
    getSurface().unbind();

    // Unbind indices
    getSurface().unbindIndices();
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMeshHexagonal
//----------------------------------------------------------------------------
RenderMeshHexagonal::RenderMeshHexagonal(units::angle::degree_t horizontalFOV, units::angle::degree_t verticalFOV,
                                         units::angle::degree_t interommatidiaAngle)
:   m_NumHorizontalHexes(ceil(horizontalFOV / interommatidiaAngle)), m_NumVerticalHexes(ceil(verticalFOV / interommatidiaAngle))
{
    using namespace units::literals;

    // Pre-calculate cos30 and sin30
    const double cos30 = units::math::cos(30_deg);
    const double sin30 = units::math::sin(30_deg);

    // Calculate side length from interommatidia angle
    const units::angle::degree_t sideLength = interommatidiaAngle / (2.0 * cos30);

    // Calculate other hexagon dimensions
    const units::angle::degree_t hexDistance = cos30 * sideLength;
    const units::angle::degree_t hexHeight = sin30 * sideLength;
    const units::angle::degree_t halfSideLength = 0.5 * sideLength;
    const units::angle::degree_t halfHexHeight = halfSideLength + hexHeight;

    // Calculate hex position offsets to build each hex's vertices from
    const units::angle::degree_t hexAngleOffsets[6][2] = {
        {0_deg, -halfHexHeight},
        {hexDistance, -halfSideLength},
        {hexDistance, halfSideLength},
        {0_deg, halfHexHeight},
        {-hexDistance, halfSideLength},
        {-hexDistance, -halfSideLength}
    };

    // Determine size of rectangles used for final output
    const float rectangleWidth = 1.0f / (float)m_NumHorizontalHexes;
    const float rectangleHeight = 1.0f / (float)m_NumVerticalHexes;
    const float halfRectangleWidth = rectangleWidth * 0.5f;

    const float hexRectangleOffsets[6][2] = {
        {halfRectangleWidth, 0.0f},
        {rectangleWidth, 0.0f},
        {rectangleWidth, rectangleHeight},
        {halfRectangleWidth, rectangleHeight},
        {0.0f, rectangleHeight},
        {0.0f, 0.0f}
    };

    // **TODO** reserve
    std::vector<GLfloat> positions;
    std::vector<GLfloat> textureCoords;
    std::vector<GLuint> indices;

    // Loop through grid of hexes
    const int numHorizontalRadiusSegments = m_NumHorizontalHexes / 2;
    const int numVerticalRadiusSegments = m_NumVerticalHexes / 2;
    for(int i = -numVerticalRadiusSegments; i < numVerticalRadiusSegments; i++) {
        for(int j = -numHorizontalRadiusSegments; j < numHorizontalRadiusSegments; j++) {
            // Calculate cartesian coordinates of centre of hexagon in "odd-r" horizontal layout
            // https://www.redblobgames.com/grids/hexagons/
            units::angle::degree_t hexLongitude = j * (2.0 * hexDistance);
            const units::angle::degree_t hexLatitude = i * (hexHeight + sideLength);

            // If row is odd, add additional distance
            if((i & 1) != 0) {
                hexLongitude += hexDistance;
            }

            // Calculate position of this hex in output rectangular grid
            const float hexX = rectangleWidth * (float)j;
            const float hexY = rectangleHeight * (float)i;

            // Cache index of first hex in
            const size_t hexStartVertexIndex = positions.size() / 2;

            // Loop through vertices
            for(unsigned int v = 0; v < 6; v++) {
                // Add positions, offsetting to centre of screen
                positions.push_back(hexX + hexRectangleOffsets[v][0]);
                positions.push_back(hexY + hexRectangleOffsets[v][1]);

                // Calculate position of vertex
                const units::angle::degree_t vertexLongitude = hexLongitude + hexAngleOffsets[v][0];
                const units::angle::degree_t vertexLatitude = hexLatitude + hexAngleOffsets[v][1];

                const GLfloat sinLatitude = sin(vertexLatitude);
                const GLfloat cosLatitude = cos(vertexLatitude);
                const GLfloat sinLongitude = sin(vertexLongitude);
                const GLfloat cosLongitude = cos(vertexLongitude);

                // Add vertex texture coordinate
                textureCoords.push_back(sinLatitude * cosLongitude);
                textureCoords.push_back(sinLongitude);
                textureCoords.push_back(cosLatitude * cosLongitude);
            }

            // Add two quads to render hexagon
            indices.push_back(hexStartVertexIndex);
            indices.push_back(hexStartVertexIndex + 1);
            indices.push_back(hexStartVertexIndex + 2);
            indices.push_back(hexStartVertexIndex + 3);

            indices.push_back(hexStartVertexIndex);
            indices.push_back(hexStartVertexIndex + 3);
            indices.push_back(hexStartVertexIndex + 4);
            indices.push_back(hexStartVertexIndex + 5);
        }
    }

    // Bind surface
    getSurface().bind();

    // Upload positions, texture coordinates and indices
    getSurface().uploadPositions(positions, 2);
    getSurface().uploadTexCoords(textureCoords, 3);
    getSurface().uploadIndices(indices, GL_QUADS);

    // Unbind surface
    getSurface().unbind();

    // Unbind indices
    getSurface().unbindIndices();
}
}   // namespace AntWorld
}   // namespace BoBRobotics
