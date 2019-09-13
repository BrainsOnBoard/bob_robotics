#include "antworld/render_target_hex_display.h"

// BoB robotics includes
#include "antworld/render_mesh.h"

// Third-party includes
#include "plog/Log.h"

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTargetHexDisplay
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
RenderTargetHexDisplay::RenderTargetHexDisplay(const RenderMeshHexagonal &renderMesh)
:   RenderTargetHexDisplay(renderMesh.getNumHorizontalHexes(), renderMesh.getNumVerticalHexes())
{
}
//----------------------------------------------------------------------------
RenderTargetHexDisplay::RenderTargetHexDisplay(const RenderMeshHexagonal &renderMeshLeft, const RenderMeshHexagonal &renderMeshRight)
:   RenderTargetHexDisplay(renderMeshLeft.getNumHorizontalHexes() + renderMeshRight.getNumHorizontalHexes(), renderMeshLeft.getNumVerticalHexes())
{
}
//----------------------------------------------------------------------------
void RenderTargetHexDisplay::render(GLint viewportX, GLint viewportY,
                                    GLsizei viewportWidth, GLsizei viewportHeight) const
{
    // Set viewport
    glViewport(viewportX, viewportY,
               viewportWidth, viewportHeight);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0,
               0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Bind render target texture
    glBindTexture(GL_TEXTURE_2D, getTexture());

     // Bind surface
    m_Surface.bind();

    // Render surface
    m_Surface.render();

    // Unbind surface
    m_Surface.unbind();

    // Unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);
}
//----------------------------------------------------------------------------
RenderTargetHexDisplay::RenderTargetHexDisplay(unsigned int numHorizontalHexes, unsigned int numVerticalHexes)
:   RenderTarget(numHorizontalHexes, numVerticalHexes)
{
    using namespace units::math;
    using namespace units::literals;

    LOGD << "Creating " << numHorizontalHexes << "x" << numVerticalHexes << " render target";

    // Pre-calculate cos30 and sin30
    const float cos30 = cos(30_deg);
    const float sin30 = sin(30_deg);

    // Calculate side lengths that would be required to fit display grid into normalized coordinate space in each dimension
    const float horizontalSideLength = 1.0f / ((float)numHorizontalHexes * 2.0f * cos30);
    const float verticalSideLength = 1.0f / ((float)numVerticalHexes * (1.0f + sin30));

    // Pick smallest
    const float sideLength = std::min(horizontalSideLength, verticalSideLength);

    // Calculate other hexagon dimensions
    const float hexDistance = cos30 * sideLength;
    const float hexHeight = sin30 * sideLength;
    const float halfSideLength = 0.5f * sideLength;
    const float halfHexHeight = halfSideLength + hexHeight;

    // Calculate hex position offsets to build each hex's vertices from
    const float hexPositionOffsets[6][2] = {
        {0.0f, halfHexHeight},
        {hexDistance, halfSideLength},
        {hexDistance, -halfSideLength},
        {0.0f, -halfHexHeight},
        {-hexDistance, -halfSideLength},
        {-hexDistance, halfSideLength}
    };

    // Determine size of rectangles used for final output
    const float rectangleWidth = 1.0f / (float)numHorizontalHexes;
    const float rectangleHeight = 1.0f / (float)numVerticalHexes;
    const float halfRectangleWidth = rectangleWidth * 0.5f;

    const float hexRectangleOffsets[6][2] = {
        {halfRectangleWidth, rectangleHeight},
        {rectangleWidth, rectangleHeight},
        {rectangleWidth, 0.0f},
        {halfRectangleWidth, 0.0f},
        {0.0f, 0.0f},
        {0.0f, rectangleHeight},
    };

    // **TODO** reserve
    std::vector<GLfloat> positions;
    std::vector<GLfloat> textureCoords;
    std::vector<GLbyte> colours;
    std::vector<GLuint> indices;

    // Loop through grid of hexes
    const int colBegin = numHorizontalHexes / 2;
    const int rowBegin = numVerticalHexes / 2;
    const int colEnd = numHorizontalHexes - colBegin;
    const int rowEnd = numVerticalHexes - rowBegin;

    const float centreU = (float)colBegin * rectangleWidth;
    const float centreV = (float)rowBegin * rectangleHeight;

    for(int i = -rowBegin; i < rowEnd; i++) {
        for(int j = -colBegin; j < colEnd; j++) {
            // Calculate cartesian coordinates of centre of hexagon in "odd-r" horizontal layout
            // https://www.redblobgames.com/grids/hexagons/
            float hexX = j * (2.0f * hexDistance);
            float hexY = i * (hexHeight + sideLength);

            // If row is odd, add additional distance
            if((i & 1) != 0) {
                hexX += hexDistance;
            }

            const float hexU = centreU + (j * rectangleWidth);
            const float hexV = centreV + (i * rectangleHeight);

            // Cache index of first hex in
            const size_t hexStartVertexIndex = positions.size() / 2;

            // Loop through vertices
            for(unsigned int v = 0; v < 6; v++) {
                // Add positions, offsetting to centre of screen
                positions.push_back(0.5f + hexX + hexPositionOffsets[v][0]);
                positions.push_back(0.5f + hexY + hexPositionOffsets[v][1]);

                // Add texture coordinates
                textureCoords.push_back(hexU + hexRectangleOffsets[v][0]);
                textureCoords.push_back(hexV + hexRectangleOffsets[v][1]);
            }

            // Add two quads to render hexagon
            indices.push_back(hexStartVertexIndex);
            indices.push_back(hexStartVertexIndex + 3);
            indices.push_back(hexStartVertexIndex + 2);
            indices.push_back(hexStartVertexIndex + 1);

            indices.push_back(hexStartVertexIndex);
            indices.push_back(hexStartVertexIndex + 5);
            indices.push_back(hexStartVertexIndex + 4);
            indices.push_back(hexStartVertexIndex + 3);
        }
    }

    // Bind surface
    m_Surface.bind();

    // Upload positions, texture coordinates and indices
    m_Surface.uploadPositions(positions, 2);
    m_Surface.uploadTexCoords(textureCoords, 2);
    m_Surface.uploadIndices(indices, GL_QUADS);

    // Unbind surface
    m_Surface.unbind();

    // Unbind indices
    m_Surface.unbindIndices();
}
}   // namespace AntWorld
}   // namespace BoBRobotics
