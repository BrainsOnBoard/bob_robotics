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
RenderMesh::RenderMesh() : m_VAO(0), m_PositionVBO(0), m_TextureCoordsVBO(0), m_IBO(0), m_NumIndices(0)
{
}
//----------------------------------------------------------------------------
RenderMesh::RenderMesh(degree_t horizontalFOV, degree_t verticalFOV, degree_t startLongitude,
                       unsigned int numHorizontalSegments, unsigned int numVerticalSegments)
{
    // We need a vertical for each segment and one extra
    const unsigned int numHorizontalVerts = numHorizontalSegments + 1;
    const unsigned int numVerticalVerts = numVerticalSegments + 1;

    // Create a vertex array object to bind everything together
    glGenVertexArrays(1, &m_VAO);

    // Bind vertex array
    glBindVertexArray(m_VAO);

    {
        // Calculate number of vertices in mesh
        const unsigned int numVertices = numHorizontalVerts * numVerticalVerts;

        // Reserve 2 XY positions and 2 SRT texture coordinates for each vertical
        std::vector<GLfloat> positions;
        std::vector<GLfloat> textureCoords;
        positions.reserve(numVertices * 2);
        textureCoords.reserve(numVertices * 2);

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

        // Generate two vertex buffer objects, one for positions and one for texture coordinates
        glGenBuffers(1, &m_PositionVBO);
        glGenBuffers(1, &m_TextureCoordsVBO);

        // Bind and upload positions buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_PositionVBO);
        glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(GLfloat), positions.data(), GL_STATIC_DRAW);

        // Set vertex pointer and enable client state in VAO
        glVertexPointer(2, GL_FLOAT, 0, BUFFER_OFFSET(0));
        glEnableClientState(GL_VERTEX_ARRAY);

        // Bind and upload texture coordinates buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_TextureCoordsVBO);
        glBufferData(GL_ARRAY_BUFFER, textureCoords.size() * sizeof(GLfloat), textureCoords.data(), GL_STATIC_DRAW);

        // Set texture coordinate pointer and enable client state in VAO
        glTexCoordPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    {
        // Calculate number of quads required to draw mesh
        m_NumIndices = numHorizontalSegments * numVerticalSegments * 4;

        // Reserce indices
        std::vector<GLuint> indices;
        indices.reserve(m_NumIndices);

        // Loop through quads
        for(unsigned int y = 0; y < numVerticalSegments; y++) {
            for(unsigned int x = 0; x < numHorizontalSegments; x++) {
                indices.push_back((y * numHorizontalVerts) + x);
                indices.push_back((y * numHorizontalVerts) + x + 1);
                indices.push_back(((y + 1) * numHorizontalVerts) + x + 1);
                indices.push_back(((y + 1) * numHorizontalVerts) + x);
            }
        }

        // Generate index buffer objects to hold primitive indices
        glGenBuffers(1, &m_IBO);

        // Bind and upload index buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_IBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);
    }

    // Unbind vertex array and array buffers
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
//----------------------------------------------------------------------------
RenderMesh::~RenderMesh()
{
    // Delete render mesh objects
    glDeleteBuffers(1, &m_PositionVBO);
    glDeleteBuffers(1, &m_TextureCoordsVBO);
    glDeleteBuffers(1, &m_IBO);
    glDeleteVertexArrays(1, &m_VAO);
}
//----------------------------------------------------------------------------
void RenderMesh::render() const
{
    // Bind render mesh VAO
    glBindVertexArray(m_VAO);

    // Draw render mesh quads
    glDrawElements(GL_QUADS, m_NumIndices, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

    // Unbind VAO
    glBindVertexArray(0);
}
}   // namespace AntWorld
}   // namespace BoBRobotics