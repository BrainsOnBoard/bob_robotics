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
    m_Surface.render(GL_QUADS);

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
        getSurface().uploadIndices(indices);
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
                                         unsigned int numHorizontalSegments, unsigned int numVerticalSegments)
{
    using namespace units::angle;
    using namespace units::math;
    using namespace units::literals;


    // Pre-calculate cos30 and sin30
    const float cos30 = cos(30_deg);
    const float sin30 = sin(30_deg);

    // Calculate side lengths that would be required to fit rendermesh into normalized coordinate space in each dimension
    const float horizontalSideLength = 1.0f / ((float)numHorizontalSegments * 2.0f * cos30);
    const float verticalSideLength = 1.0f / ((float)numVerticalSegments * (1.0f + sin30));

    // Pick smallest
    const float sideLength = std::min(horizontalSideLength, verticalSideLength);

    // Calculate other hexagon dimensions
    const float hexDistance = cos30 * sideLength;
    const float hexHeight = sin30 * sideLength;

    // Calculate hex position offsets to build each hex's vertices from
    /*float halfSideLength = 0.5f * m_SideLength;
    float halfHexHeight = halfSideLength + GetHexHeight();
    return new Vector3[6]
    {
        new Vector3(0.0f, 0.0f, -halfHexHeight),
        new Vector3(hexDistance, 0.0f, -halfSideLength),
        new Vector3(hexDistance, 0.0f, halfSideLength),
        new Vector3(0.0f, 0.0f, halfHexHeight),
        new Vector3(-hexDistance, 0.0f, halfSideLength),
        new Vector3(-hexDistance, 0.0f, -halfSideLength),
    };*/
    // **TODO** reserve
    std::vector<GLfloat> positions;
    std::vector<GLfloat> textureCoords;
    std::vector<GLuint> indices;

    size_t vertexIndex = 0;
    size_t triangleIndex = 0;

    // Loop through grid of hexes
    const int numHorizontalRadiusSegments = numHorizontalSegments / 2;
    const int numVerticalRadiusSegments = numVerticalSegments / 2;
    for(int i = -numVerticalRadiusSegments; i < numVerticalRadiusSegments; i++) {
        for(int j = -numHorizontalRadiusSegments; j < numHorizontalRadiusSegments; j++) {
            // Calculate cartesian coordinates of centre of hexagon
            // **TODO** link to ole amit's hex site
            float hexX = j * (2.0f * hexDistance);
            float hexY = i * (hexHeight + sideLength);

            // If row is odd, add additional distance
            if((row & 1) != 0) {
                hexX += hexDistance;
            }

            // Calcualate distance to hex to origin - this is treated as the arc length (radius = 1)
            const float arcLength = std::sqrt((hexX * hexX) + (hexY * hexY)));

            // Calculate T coordinate
            const float hexT = cos(radian_t(arcLength));
            BOB_ASSERT(hexT >= 0.0f);

            const float a = sin(radian_t(arcLength)) / arcLength;
            const float hexS = hexX * a;
            const float hexR = hexY * a;

            // Calculate it's modelspace position
            int firstFillVertexIndex = vertexIndex;

            // Loop through vertices
            for(unsigned int v = 0; v < 6; v++) {
                // Assign components
                positions[vertexIndex] = hexPosition + hexPositionOffsets[vertex];
                uvs[vertexIndex] = hexUVs[vertex];

                // Next vertex
                vertexIndex++;
                // Add vertex to mesh
                AddHexVertex(vertex, hexPosition,
                    hexUVs, hexPositionOffsets,
                    positions, colours, uvs,
                    ref vertexIndex);

                // Add a triangle for all but 1st and last vertex
                if(vertex > 0 && vertex < 5)
                {
                    triangleIndices[triangleIndex++] = firstFillVertexIndex;
                    triangleIndices[triangleIndex++] = firstFillVertexIndex + vertex + 1;
                    triangleIndices[triangleIndex++] = firstFillVertexIndex + vertex;
                }
            }
        }
    }
    // Start generating hexes
}
}   // namespace AntWorld
}   // namespace BoBRobotics
