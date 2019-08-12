#include "antworld/render_target_hex_display.h"

// BoB robotics includes
#include "antworld/render_mesh.h"

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTargetHexDisplay
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
RenderTargetHexDisplay::RenderTargetHexDisplay(const RenderMeshHexagonal &renderMesh)
:   RenderTarget(renderMesh.getNumHorizontalHexes(), renderMesh.getNumVerticalHexes())
{
    using namespace units::math;
    using namespace units::literals;

    // Pre-calculate cos30 and sin30
    const float cos30 = cos(30_deg);
    const float sin30 = sin(30_deg);

    // Calculate side lengths that would be required to fit display grid into normalized coordinate space in each dimension
    const float horizontalSideLength = 1.0f / ((float)renderMesh.getNumHorizontalHexes() * 2.0f * cos30);
    const float verticalSideLength = 1.0f / ((float)renderMesh.getNumVerticalHexes() * (1.0f + sin30));

    // Pick smallest
    const float sideLength = std::min(horizontalSideLength, verticalSideLength);

    // Calculate other hexagon dimensions
    const float hexDistance = cos30 * sideLength;
    const float hexHeight = sin30 * sideLength;
    const float halfSideLength = 0.5f * sideLength;
    const float halfHexHeight = halfSideLength + hexHeight;

    // Calculate hex position offsets to build each hex's vertices from
    const float hexPositionOffsets[6][2] = {
        {0.0f, -halfHexHeight},
        {hexDistance, -halfSideLength},
        {hexDistance, halfSideLength},
        {0.0f, halfHexHeight},
        {-hexDistance, halfSideLength},
        {-hexDistance, -halfSideLength}
    };

    // Determine size of rectangles used for final output
    const float rectangleWidth = 1.0f / (float)renderMesh.getNumHorizontalHexes();
    const float rectangleHeight = 1.0f / (float)renderMesh.getNumVerticalHexes();
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
    const int numHorizontalRadiusSegments = renderMesh.getNumHorizontalHexes() / 2;
    const int numVerticalRadiusSegments = renderMesh.getNumVerticalHexes() / 2;
    for(int i = -numVerticalRadiusSegments; i < numVerticalRadiusSegments; i++) {
        for(int j = -numHorizontalRadiusSegments; j < numHorizontalRadiusSegments; j++) {
            // Calculate cartesian coordinates of centre of hexagon in "odd-r" horizontal layout
            // https://www.redblobgames.com/grids/hexagons/
            float hexX = j * (2.0f * hexDistance);
            float hexY = i * (hexHeight + sideLength);

            // If row is odd, add additional distance
            if((i & 1) != 0) {
                hexX += hexDistance;
            }

            const float hexU = j * rectangleWidth;
            const float hexV = i * rectangleHeight;

            // Cache index of first hex in
            const size_t hexStartVertexIndex = positions.size() / 2;

            // Loop through vertices
            for(unsigned int v = 0; v < 6; v++) {
                // Add positions, offsetting to centre of screen
                positions.push_back(0.5f + hexX + hexPositionOffsets[v][0]);
                positions.push_back(0.5f + hexY + hexPositionOffsets[v][1]);

                // Add texture coordinates
                textureCoords.push_back(0.5f + hexU + hexRectangleOffsets[v][0]);
                textureCoords.push_back(0.5f + hexV + hexRectangleOffsets[v][1]);
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
//----------------------------------------------------------------------------
void RenderTargetHexDisplay::render() const
{
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
}   // namespace AntWorld
}   // namespace BoBRobotics
