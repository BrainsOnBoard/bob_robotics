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

//---------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMesh::Border
//----------------------------------------------------------------------------
RenderMesh::Border::Border(bool flipAzimuth, degree_t horizontalFOV, degree_t verticalFOV, degree_t centreAzimuth, degree_t centreElevation)
:   m_MinElevation(centreElevation - (verticalFOV / 2.0)), m_MaxElevation(centreElevation + (verticalFOV / 2.0))
{
    if(flipAzimuth) {
        m_MinAzimuth = -centreAzimuth - (horizontalFOV / 2.0);
        m_MaxAzimuth = -centreAzimuth + (horizontalFOV / 2.0);
    }
    else {
        m_MinAzimuth = centreAzimuth - (horizontalFOV / 2.0);
        m_MaxAzimuth = centreAzimuth + (horizontalFOV / 2.0);
    }
}
//----------------------------------------------------------------------------
RenderMesh::Border::Border(bool flipAzimuth, const std::string &eyeBorderFilename)
{
    std::ifstream is(eyeBorderFilename, std::ifstream::binary);
    if(!is.good()) {
        throw std::runtime_error("Unable to open eye border file name " + eyeBorderFilename);
    }

    // Get length of eye border file, hence number of vertices
    is.seekg(0, is.end);
    const auto numEyeBorderBytes = is.tellg();
    const auto numEyeBorderVertices = numEyeBorderBytes / (sizeof(double) * 2);
    LOGI << "Read " << numEyeBorderVertices << " eye border vertices";
    is.seekg(0, is.beg);

    // Allocate vector, read in data from file
    m_EyeBorderVertices.resize(numEyeBorderVertices);
    is.read(reinterpret_cast<char*>(m_EyeBorderVertices.data()), sizeof(double) * 2 * numEyeBorderVertices);

    // Flip all vertices azimuth if specified
    if(flipAzimuth) {
        for(auto &v : m_EyeBorderVertices) {
            std::get<1>(v) = -std::get<1>(v);
        }
    }
    // Calculate bounds in terms of azimuth and elevation
    m_MinAzimuth = degree_t(std::numeric_limits<double>::max());
    m_MaxAzimuth = degree_t(std::numeric_limits<double>::lowest());
    m_MinElevation = degree_t(std::numeric_limits<double>::max());
    m_MaxElevation = degree_t(std::numeric_limits<double>::lowest());
    for(const auto &v : m_EyeBorderVertices) {
        m_MinAzimuth = min(m_MinAzimuth, std::get<1>(v));
        m_MaxAzimuth = max(m_MaxAzimuth, std::get<1>(v));
        m_MinElevation = min(m_MinElevation, std::get<0>(v));
        m_MaxElevation = max(m_MaxElevation, std::get<0>(v));
    }

    LOGI << "Eye border azimuth: " << m_MinAzimuth.value() << "-" << m_MaxAzimuth.value() << ", elevation: " << m_MinElevation.value() << "-" << m_MaxElevation.value();

}
//----------------------------------------------------------------------------
bool RenderMesh::Border::isInEye(degree_t azimuth, degree_t elevation) const
{
    // If elevation and azimuth are within bounding box
    if(azimuth >= m_MinAzimuth && azimuth < m_MaxAzimuth &&
        elevation >= m_MinElevation && elevation < m_MaxElevation)
    {
        // If there's no eye border vertices, return true
        if(m_EyeBorderVertices.empty()) {
            return true;
        }
        // Otherwise
        else {
            // Loop over all edges in the polygon.
            bool inside = false;
            for(size_t i = 0; i < m_EyeBorderVertices.size(); i++) {
                const auto &a = m_EyeBorderVertices[i];
                const auto &b = m_EyeBorderVertices[(i + 1) % m_EyeBorderVertices.size()];

                // If edge straddles ray in y, they might intersect
                if((std::get<1>(a) >= azimuth) != (std::get<1>(b) >= azimuth)) {
                    // If edge's X values both right of the point, must hit
                    if((std::get<0>(a) >= elevation) && (std::get<0>(b) >= elevation)){
                        inside = !inside;
                    }
                    // Otherwise, compute intersection with +X ray
                    else if((std::get<0>(b) - (std::get<1>(b) - azimuth) * (std::get<0>(a) - std::get<0>(b)) / (std::get<1>(a) - std::get<1>(b))) >= elevation) {
                        inside = !inside;
                    }
                }
            }

            return inside;
        }
    }
    // Otherwise, if azimuth and elevation are out of bounds, return false
    else {
        return false;
    }
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMeshSpherical
//----------------------------------------------------------------------------
RenderMeshSpherical::RenderMeshSpherical(bool flipAzimuth, degree_t horizontalFOV, degree_t verticalFOV, degree_t startElevation,
                                         unsigned int numHorizontalSegments, unsigned int numVerticalSegments)
:   RenderMeshSpherical(Border(flipAzimuth, horizontalFOV, verticalFOV, 0_deg, startElevation + (verticalFOV / 2.0)),
                        numHorizontalSegments, numVerticalSegments)
{
}
//----------------------------------------------------------------------------
RenderMeshSpherical::RenderMeshSpherical(bool flipAzimuth, const std::string &eyeBorderFilename,
                                         unsigned int numHorizontalSegments, unsigned int numVerticalSegments)
:   RenderMeshSpherical(Border(flipAzimuth, eyeBorderFilename), numHorizontalSegments, numVerticalSegments)
{
}
//----------------------------------------------------------------------------
RenderMeshSpherical::RenderMeshSpherical(const Border &border, unsigned int numHorizontalSegments, unsigned int numVerticalSegments)
{
    // Vectors to temporarily hold 2 XY positions and 2 SRT texture coordinates for each vertex
    std::vector<GLfloat> positions;
    std::vector<GLfloat> textureCoords;

    const float segmentWidth = 1.0f / (float)numHorizontalSegments;
    const degree_t azimuthStep = (border.getMaxAzimuth() - border.getMinAzimuth()) / numHorizontalSegments;

    const float segmentHeight = 1.0f / (float)numVerticalSegments;
    const degree_t elevationStep = (border.getMaxElevation() - border.getMinElevation()) / numVerticalSegments;

    // Loop through vertices
    for(unsigned int j = 0; j < numVerticalSegments; j++) {
        // Calculate screenspace segment y position
        const float y = segmentHeight * (float)j;

        // Calculate angle of horizontal and calculate its sin and cos
        const degree_t elevation = border.getMinElevation() + (elevationStep * j);

        for(unsigned int i = 0; i < numHorizontalSegments; i++) {
            // Calculate screenspace segment x position
            const float x = segmentWidth * (float)i;

            // Calculate angle of vertical and calculate it's sin and cos
            const degree_t azimuth = border.getMinAzimuth() + (azimuthStep * i);

            // Determine angles for each vertex in quad
            std::array<std::tuple<degree_t, degree_t, float, float>, 4> vertices{
                std::make_tuple(azimuth,               elevation,                  x,                  y),
                std::make_tuple(azimuth + azimuthStep, elevation,                  x + segmentWidth,   y),
                std::make_tuple(azimuth + azimuthStep, elevation + elevationStep,  x + segmentWidth,   y + segmentHeight),
                std::make_tuple(azimuth,               elevation + elevationStep,  x,                  y + segmentHeight)};

            // If any vertices are within eye region
            if(std::any_of(vertices.cbegin(), vertices.cend(),
                [&border](const auto &v)
                {
                    return border.isInEye(std::get<0>(v), std::get<1>(v));
                }))
            {
                // Loop through vertices
                for(const auto &v : vertices) {
                    const GLfloat sinElevation = sin(std::get<1>(v));
                    const GLfloat cosElevation = cos(std::get<1>(v));
                    const GLfloat sinAzimuth = sin(std::get<0>(v));
                    const GLfloat cosAzimuth = cos(std::get<0>(v));

                    // Add vertex position
                    positions.push_back(std::get<2>(v));
                    positions.push_back(std::get<3>(v));

                    // Add vertex texture coordinate
                    textureCoords.push_back(sinAzimuth * cosElevation);
                    textureCoords.push_back(sinElevation);
                    textureCoords.push_back(cosAzimuth * cosElevation);
                }
            }

        }
    }

     // Bind surface
    getSurface().bind();

    // Upload positions and texture coordinates
    getSurface().uploadPositions(positions, 2);
    getSurface().uploadTexCoords(textureCoords, 3);
    getSurface().setPrimitiveType(GL_QUADS);

    // Unbind surface
    getSurface().unbind();
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderMeshHexagonal
//----------------------------------------------------------------------------
RenderMeshHexagonal::RenderMeshHexagonal(bool flipAzimuth, degree_t horizontalFOV, degree_t verticalFOV,
                                         degree_t centreAzimuth, degree_t centreElevation,
                                         units::angle::degree_t interommatidiaAngle)
:   RenderMeshHexagonal(Border(flipAzimuth, horizontalFOV, verticalFOV, centreAzimuth, centreElevation), interommatidiaAngle)
{
}
//----------------------------------------------------------------------------
RenderMeshHexagonal::RenderMeshHexagonal(bool flipAzimuth, const std::string &eyeBorderFilename,
                                         units::angle::degree_t interommatidiaAngle)
:   RenderMeshHexagonal(Border(flipAzimuth, eyeBorderFilename), interommatidiaAngle)
{
}
//----------------------------------------------------------------------------
RenderMeshHexagonal::RenderMeshHexagonal(const Border &border, units::angle::degree_t interommatidiaAngle)
:   m_NumHorizontalHexes(ceil(border.getHorizontalFOV() / interommatidiaAngle)),
    m_NumVerticalHexes(ceil(border.getVerticalFOV() /  interommatidiaAngle))
{
    using namespace units::literals;
    using namespace units::angle;

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
            units::angle::degree_t hexAzimuth = border.getCentreAzimuth() + (j * (2.0 * hexDistance));
            const units::angle::degree_t hexElevation = border.getCentreElevation() + (i * (hexHeight + sideLength));

            // If row is odd, add additional distance
            if((i & 1) != 0) {
                hexAzimuth += hexDistance;
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
                const units::angle::degree_t vertexAzimuth = hexAzimuth + hexAngleOffsets[v][0];
                const units::angle::degree_t vertexElevation = hexElevation + hexAngleOffsets[v][1];

                const GLfloat sinElevation = sin(vertexElevation);
                const GLfloat cosElevation = cos(vertexElevation);
                const GLfloat sinAzimuth = sin(vertexAzimuth);
                const GLfloat cosAzimuth = cos(vertexAzimuth);

                // Add vertex texture coordinate
                textureCoords.push_back(sinAzimuth * cosElevation);
                textureCoords.push_back(sinElevation);
                textureCoords.push_back(cosAzimuth * cosElevation);
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
    getSurface().setPrimitiveType(GL_QUADS);

    // Unbind surface
    getSurface().unbind();

    // Unbind indices
    getSurface().unbindIndices();
}
}   // namespace AntWorld
}   // namespace BoBRobotics
