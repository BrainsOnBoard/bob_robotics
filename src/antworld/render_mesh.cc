// BoB robotics includes
#include "antworld/common.h"
#include "antworld/render_mesh.h"

// Standard C++ includes
#include <vector>

// Standard C includes
#include <cassert>

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
// BoBRobotics::AntWorld::RenderMeshCubeMap
//----------------------------------------------------------------------------
RenderMeshCubeMap::RenderMeshCubeMap()
{
    // Bind surface
    getSurface().bind();
    
    // Build vertices
    {
        // Reserve 2 XY positions and 2 SRT texture coordinates for each of the 12 vertices
        const std::vector<GLfloat> positions{
            // +Y
            0.25f, 1.00f,
            0.25f, 0.6666667f,
            0.50f, 0.6666667f,
            0.50f, 1.00f,

            // -X
            0.00f, 0.6666667f,
            0.00f, 0.3333333f,
            0.25f, 0.3333333f,
            0.25f, 0.6666667f,

            // +Z
            0.25f, 0.6666667f,
            0.25f, 0.3333333f,
            0.50f, 0.3333333f,
            0.50f, 0.6666667f,

            // +X
            0.50f, 0.6666667f,
            0.50f, 0.3333333f,
            0.75f, 0.3333333f,
            0.75f, 0.6666667f,

            // -Z
            0.75f, 0.6666667f,
            0.75f, 0.3333333f,
            1.00f, 0.3333333f,
            1.00f, 0.6666667f,

            // -Y
            0.25f, 0.3333333f,
            0.25f, 0.00f,
            0.50f, 0.00f,
            0.50f, 0.3333333f};

        const std::vector<GLfloat> textureCoords{
            // +Y
            -1.0f,  -1.0f, -1.0f,
            -1.0f,  -1.0f,  1.0f,
            1.0f,  -1.0f,  1.0f,
            1.0f,  -1.0f, -1.0f,

            // -X
            -1.0f,  -1.0f,  -1.0f,
            -1.0f,  1.0f,  -1.0f,
            -1.0f,  1.0f,  1.0f,
            -1.0f,  -1.0f,  1.0f,

            // +Z
            -1.0f,  -1.0f,  1.0f,
            -1.0f, 1.0f,  1.0f,
            1.0f, 1.0f,  1.0f,
            1.0f,  -1.0f,  1.0f,

            // +X
            1.0f,  -1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  -1.0f,
            1.0f,  -1.0f,  -1.0f,

            // -Z
            1.0f, -1.0f, -1.0f,
            1.0f,  1.0f, -1.0f,
            -1.0f, 1.0f, -1.0f,
            -1.0f,  -1.0f, -1.0f,

            // -Y
            -1.0f, 1.0f,  1.0f,
            -1.0f, 1.0f, -1.0f,
            1.0f, 1.0f, -1.0f,
            1.0f, 1.0f,  1.0f};

        assert(positions.size() == (6 * 4 * 2));
        assert(textureCoords.size() == (6 * 4 * 3));

         // Upload positions and texture coordinates
        getSurface().uploadPositions(positions, 2);
        getSurface().uploadTexCoords(textureCoords, 3);
    }

    {
        const std::vector<GLuint> indices{
            0,  1,  2,  3,      // +Y
            4,  5,  6,  7,      // -X
            8,  9, 10, 11,      // +Z
            12, 13, 14, 15,     // +X
            16, 17, 18, 19,     // -Z
            20, 21, 22, 23};    // -Y
        
        assert(indices.size() == (6 * 4));

        // Upload indices to surface
        getSurface().uploadIndices(indices);
    }

    // Unbind surface
    getSurface().unbind();

    // Unbind indices
    getSurface().unbindIndices();
}

}   // namespace AntWorld
}   // namespace BoBRobotics


