// BoB robotics includes
#include "common/macros.h"

// BoB robotics includes
#include "antworld/common.h"
#include "antworld/render_mesh.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <algorithm>
#include <fstream>
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
RenderMeshHexagonal::RenderMeshHexagonal(const std::string &eyeBorderFilename, units::angle::degree_t interommatidiaAngle)
{
    using namespace units::literals;
    using namespace units::angle;

    // Load eye border file
    const auto eyeBorder = loadEyeBorder(eyeBorderFilename);


    const auto elevationRange = std::minmax_element(eyeBorder.cbegin(), eyeBorder.cend(),
                                                    [](const std::tuple<float, float> &a, const std::tuple<float, float> &b)
                                                    {
                                                        return (std::get<0>(a) < std::get<0>(b));
                                                    });
    const auto azimuthRange = std::minmax_element(eyeBorder.cbegin(), eyeBorder.cend(),
                                                  [](const std::tuple<float, float> &a, const std::tuple<float, float> &b)
                                                  {
                                                    return (std::get<1>(a) < std::get<1>(b));
                                                  });
    LOGI << "Eye border angles (" << elevationRange.first << ", " << *azimuthRange.first << ") - (" << elevationRange.second << ", " << azimuthRange.second << ")";

    assert(false);

    // Pre-calculate cos30 and sin30
    const double cos30 = units::math::cos(30_deg);
    const double sin30 = units::math::sin(30_deg);

    // Calculate side length from interommatidia angle
    // sidelength is 'radius' i.e. distance from any vertex to centre and interommatidia angle is 'diameter'
    const units::angle::degree_t sideLength = interommatidiaAngle / 2.0;

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

    // Positions used to map hexagons to rectangles
    // **NOTE** quality of mapping is not SUPER-important as the resultant rectangle will be averaged
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
    std::vector<GLuint> indices;

    // Loop through grid of hexes
    const int colBegin = m_NumHorizontalHexes / 2;
    const int rowBegin = m_NumVerticalHexes / 2;
    const int colEnd = m_NumHorizontalHexes - colBegin;
    const int rowEnd = m_NumVerticalHexes - rowBegin;

    const float centreX = (float)colBegin * rectangleWidth;
    const float centreY = (float)rowBegin * rectangleHeight;

    for(int i = -rowBegin; i < rowEnd; i++) {
        for(int j = -colBegin; j < colEnd; j++) {
            // Calculate cartesian coordinates of centre of hexagon in "odd-r" horizontal layout
            // https://www.redblobgames.com/grids/hexagons/
            units::angle::degree_t hexLongitude = j * (2.0 * hexDistance);
            const units::angle::degree_t hexLatitude = i * (hexHeight + sideLength);

            // If row is odd, add additional distance
            if((i & 1) != 0) {
                hexLongitude += hexDistance;
            }

            // Calculate position of this hex in output rectangular grid, offsetting to centre of screen
            const float hexX = centreX + (rectangleWidth * (float)j);
            const float hexY = centreY - (rectangleHeight * (float)i);

            // Cache index of first hex in
            const size_t hexStartVertexIndex = positions.size() / 2;

            // Loop through vertices
            for(unsigned int v = 0; v < 6; v++) {
                // Add positions
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
                textureCoords.push_back(sinLongitude * cosLatitude);
                textureCoords.push_back(sinLatitude);
                textureCoords.push_back(cosLongitude * cosLatitude);
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

    BOB_ASSERT(indices.size() == (8 * m_NumHorizontalHexes * m_NumVerticalHexes));

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
//------------------------------------------------------------------------
std::vector<std::tuple<float, float>> RenderMeshHexagonal::loadEyeBorder(const std::string &eyeBorderFilename) const
{
    std::ifstream is(eyeBorderFilename, std::ifstream::binary);
    if(!is.good()) {
        throw std::runtime_error("Unable to opne eye border file name " + eyeBorderFilename);
    }

    // Get length of eye border file, hence number of vertices
    is.seekg (0, is.end);
    const auto numEyeBorderBytes = is.tellg();
    const auto numEyeBorderVertices = numEyeBorderBytes / (sizeof(float) * 2);
    LOGI << "Read " << numEyeBorderVertices << " eye border vertices";
    is.seekg (0, is.beg);

    // Allocate vector, read in data from file and return
    std::vector<std::tuple<float, float>> eyeBorder(numEyeBorderVertices);
    is.read(reinterpret_cast<char*>(eyeBorder.data()), sizeof(float) * 2 * numEyeBorderVertices);
    return eyeBorder;
}
}   // namespace AntWorld
}   // namespace BoBRobotics
