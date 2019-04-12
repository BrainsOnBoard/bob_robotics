// BoB robotics includes
#include "antworld/common.h"
#include "antworld/world.h"
#include "common/assert.h"
#include "common/logging.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <tuple>

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
template<unsigned int N>
void readVector(std::istringstream &stream, std::vector<GLfloat> &vector, float scale)
{
    // Read components and push back
    GLfloat x;
    for(unsigned int i = 0; i < N; i++) {
        stream >> x;
        vector.push_back(x * scale);
    }
}
//----------------------------------------------------------------------------
void readFace(std::istringstream &lineStream,
              const std::vector<GLfloat> &rawPositions,
              const std::vector<GLfloat> &rawTexCoords,
              std::tuple<std::string, std::vector<GLfloat>, std::vector<GLfloat>> &currentObjSurface)
{
    // Get references to current material's positions and texture coordinates
    auto &surfacePositions = std::get<1>(currentObjSurface);
    auto &surfaceTexCoords = std::get<2>(currentObjSurface);

    // Loop through face vertex
    std::string faceIndexString;
    std::string indexString;
    for(unsigned int v = 0; v < 3; v++) {
        // Read indices i.e. P/T/N into string
        lineStream >> faceIndexString;

        // Convert into stream for processing
        std::istringstream faceIndexStream(faceIndexString);

        // Extract index of raw position and texture coordinate
        // **NOTE** obj indices start from 1
        std::getline(faceIndexStream, indexString, '/');
        const int position = stoi(indexString) - 1;
        std::getline(faceIndexStream, indexString, '/');
        const int texCoord = stoi(indexString) - 1;

        // Copy raw positions and texture coordinates into material
        std::copy_n(&rawPositions[3 * position], 3, std::back_inserter(surfacePositions));
        std::copy_n(&rawTexCoords[2 * texCoord], 2, std::back_inserter(surfaceTexCoords));
    }

    // Check this is the end of the linestream i.e. there aren't extra components
    BOB_ASSERT(lineStream.eof());
}
//----------------------------------------------------------------------------
void stripWindowsLineEnding(std::string &lineString)
{
    // If line has a Windows line ending, remove it
    if(!lineString.empty() && lineString.back() == '\r') {
        lineString.pop_back();
    }
}
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::World
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
void World::load(const std::string &filename, const GLfloat (&worldColour)[3],
                 const GLfloat (&groundColour)[3])
{
    // Create single surface
    m_Surfaces.clear();
    m_Surfaces.emplace_back();
    auto &surface = m_Surfaces.back();

    // Open file for binary IO
    std::ifstream input(filename, std::ios::binary);
    if(!input.good()) {
        throw std::runtime_error("Cannot open world file:" + filename);
    }

    // Seek to end of file, get size and rewind
    input.seekg(0, std::ios_base::end);
    const auto numTriangles = static_cast<size_t>(input.tellg()) / (sizeof(double) * 12);
    input.seekg(0);
    LOG_INFO << "World has " << numTriangles << " triangles";

    // Bind surface vertex array
    surface.bind();

    {
        // Reserve 3 XYZ positions for each triangle and 6 for the ground
        std::vector<GLfloat> positions((6 + (numTriangles * 3)) * 3);

        // Initialise bounds to limits of underlying data types
        std::fill_n(&m_MinBound[0], 3, std::numeric_limits<meter_t>::max());
        std::fill_n(&m_MaxBound[0], 3, std::numeric_limits<meter_t>::min());

        // Loop through components(X, Y and Z)
        for(unsigned int c = 0; c < 3; c++) {
            // Loop through vertices in each triangle
            for(unsigned int v = 0; v < 3; v++) {
                // Loop through triangles
                for(unsigned int t = 0; t < numTriangles; t++) {
                    // Read triangle position component
                    double trianglePosition;
                    input.read(reinterpret_cast<char*>(&trianglePosition), sizeof(double));

                    // Copy three coordinates from triangle into correct place in vertex array
                    // **NOTE** after first ground polygons
                    positions[18 + (t * 9) + (v * 3) + c] = (GLfloat)trianglePosition;

                    // Update bounds
                    m_MinBound[c] = units::math::min(m_MinBound[c], meter_t(trianglePosition));
                    m_MaxBound[c] = units::math::max(m_MaxBound[c], meter_t(trianglePosition));
                }
            }
        }

        // Add first ground plane triangle vertex positions
        positions[0] = m_MinBound[0].value();   positions[1] = m_MinBound[1].value();   positions[2] = 0.0f;
        positions[3] = m_MaxBound[0].value();   positions[4] = m_MaxBound[1].value();   positions[5] = 0.0f;
        positions[6] = m_MinBound[0].value();   positions[7] = m_MaxBound[1].value();   positions[8] = 0.0f;

        // Add second ground plane triangle vertex positions
        positions[9] = m_MinBound[0].value();   positions[10] = m_MinBound[1].value();  positions[11] = 0.0f;
        positions[12] = m_MaxBound[0].value();  positions[13] = m_MinBound[1].value();  positions[14] = 0.0f;
        positions[15] = m_MaxBound[0].value();  positions[16] = m_MaxBound[1].value();  positions[17] = 0.0f;

        // Upload positions
        surface.uploadPositions(positions);

        LOG_INFO << "Min: (" << m_MinBound[0] << ", " << m_MinBound[1] << ", " << m_MinBound[2] << ")";
        LOG_INFO << "Max: (" << m_MaxBound[0] << ", " << m_MaxBound[1] << ", " << m_MaxBound[2] << ")";
    }

    {
        // Reserve 3 RGB colours for each triangle and for the ground
        std::vector<GLfloat> colours((6 + (numTriangles * 3)) * 3);

        // Ground triangle colours
        for(unsigned int c = 0; c < (6 * 3); c += 3) {
            colours[c] = groundColour[0];
            colours[c + 1] = groundColour[1];
            colours[c + 2] = groundColour[2];
        }

        // Loop through triangles
        for(unsigned int t = 0; t < numTriangles; t++) {
            // Read triangle colour component
            // **NOTE** we only bother reading the R channel because colours are greyscale anyway
            double triangleColour;
            input.read(reinterpret_cast<char*>(&triangleColour), sizeof(double));

            // Loop through vertices that make up triangle and
            // set to world colour multiplied by triangle colour
            for(unsigned int v = 0; v < 3; v++) {
                colours[18 + (t * 9) + (v * 3)] = worldColour[0] * triangleColour;
                colours[18 + (t * 9) + (v * 3) + 1] = worldColour[1] * triangleColour;
                colours[18 + (t * 9) + (v * 3) + 2] = worldColour[2] * triangleColour;
            }
        }

        // Upload colours
        surface.uploadColours(colours);
    }

    // Unbind surface
    surface.unbind();
}
//----------------------------------------------------------------------------
void World::loadObj(const std::string &filename, float scale, int maxTextureSize, GLint textureFormat)
{
    // Get HARDWARE max texture size
    int hardwareMaxTextureSize = 0;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &hardwareMaxTextureSize);

    // If no max texture size is specified, use hardware maximum
    if(maxTextureSize == -1) {
        maxTextureSize = hardwareMaxTextureSize;
    }
    // Otherwise, use lowest of hardware or user-specified max texture size
    else {
        maxTextureSize = std::min(maxTextureSize, hardwareMaxTextureSize);
    }

    LOG_DEBUG << "Max texture size: " << maxTextureSize;

    // Vector of geometry associated with each named surface (in unindexed triangle format)
    std::vector<std::tuple<std::string, std::vector<GLfloat>, std::vector<GLfloat>>> objSurfaces;

    // Map of material names to texture indices
    std::map<std::string, Texture*> textureNames;

    // Parser
    {
        // Vectors to hold 'raw' positions and texture coordinates read from obj
        std::vector<GLfloat> rawPositions;
        std::vector<GLfloat> rawTexCoords;

        // Open obj file
        std::ifstream objFile(filename);
        if(!objFile.good()) {
            throw std::runtime_error("Cannot open obj file: " + filename);
        }

        // Get base path to load materials etc relative to
        const auto basePath = filesystem::path(filename).make_absolute().parent_path();

        // Read lines into strings
        std::string lineString;
        std::string commandString;
        std::string parameterString;
        while(std::getline(objFile, lineString)) {
            // Strip windows line endings
            stripWindowsLineEnding(lineString);

            // Entirely skip comment or empty lines
            if(lineString[0] == '#' || lineString.empty()) {
                continue;
            }

            // Wrap line in stream for easier parsing
            std::istringstream lineStream(lineString);

            // Read command from first token
            lineStream >> commandString;
            if(commandString == "mtllib") {
                lineStream >> parameterString;

                // Parse materials
                loadMaterials(basePath, parameterString,
                              textureFormat, maxTextureSize,
                              textureNames);
            }
            else if(commandString == "o") {
                lineStream >> parameterString;
                LOG_DEBUG << "Reading object: " << parameterString;
            }
            else if(commandString == "v") {
                // Read vertex
                readVector<3>(lineStream, rawPositions, scale);
            }
            else if(commandString == "vt") {
                // Read texture coordinate and check there's no unhandled components following it
                readVector<2>(lineStream, rawTexCoords, 1.0f);
                BOB_ASSERT(lineStream.eof())
            }
            else if(commandString == "vn") {
                // ignore vertex normals for now
            }
            else if(commandString == "usemtl") {
                lineStream >> parameterString;
                LOG_INFO << "\tReading surface: " << parameterString;
                objSurfaces.emplace_back(parameterString, std::initializer_list<GLfloat>(), std::initializer_list<GLfloat>());
            }
            else if(commandString == "s") {
                // ignore smoothing
            }
            else if(commandString == "f") {
                // Check that a surface has been begun
                BOB_ASSERT(!objSurfaces.empty());

                // Read face
                readFace(lineStream, rawPositions, rawTexCoords, objSurfaces.back());
            }
            else {
                LOG_WARNING << "Unhandled obj tag '" << commandString << "'";
            }

        }

        LOG_INFO << "\t" << rawPositions.size() / 3 << " raw positions, " << rawTexCoords.size() / 2 << " raw texture coordinates, ";
        LOG_INFO << objSurfaces.size() << " surfaces, " << m_Textures.size() << " textures";

        // Initialise bounds to limits of underlying data types
        std::fill_n(&m_MinBound[0], 3, std::numeric_limits<meter_t>::max());
        std::fill_n(&m_MaxBound[0], 3, std::numeric_limits<meter_t>::min());
        for(unsigned int i = 0; i < rawPositions.size(); i += 3) {
            for(unsigned int c = 0; c < 3; c++) {
                m_MinBound[c] = units::math::min(m_MinBound[c], meter_t(rawPositions[i + c]));
                m_MaxBound[c] = units::math::max(m_MaxBound[c], meter_t(rawPositions[i + c]));
            }
        }

        LOG_INFO << "Min: (" << m_MinBound[0] << ", " << m_MinBound[1] << ", " << m_MinBound[2] << ")";
        LOG_INFO << "Max: (" << m_MaxBound[0] << ", " << m_MaxBound[1] << ", " << m_MaxBound[2] << ")";
    }

    // Remove any existing surfaces
    m_Surfaces.clear();

    // Allocate new materials array to match those found in obj
    m_Surfaces.resize(objSurfaces.size());

    // Loop through surfaces
    for(unsigned int s = 0; s < objSurfaces.size(); s++) {
        const auto &objSurface = objSurfaces[s];
        auto &surface = m_Surfaces[s];

        // Bind material
        surface.bind();

        // Upload positions and texture coordinates from obj file
        surface.uploadPositions(std::get<1>(objSurface));
        surface.uploadTexCoords(std::get<2>(objSurface));

        // Find texture corresponding to this surface
        const auto tex = textureNames.find(std::get<0>(objSurface));
        if(tex != textureNames.end()) {
            surface.setTexture(tex->second);
        }

        // Unbind surface
        surface.unbind();
    }
}
//----------------------------------------------------------------------------
void World::render() const
{
    // Bind and render each material
    for(auto &surf : m_Surfaces) {
        surf.bind();
        surf.render();
        surf.unbind();
    }
}
//----------------------------------------------------------------------------
void World::loadMaterials(const filesystem::path &basePath, const std::string &filename,
                          GLint textureFormat, int maxTextureSize,
                          std::map<std::string, Texture*> &textureNames)
{
    // Open obj file
    std::ifstream mtlFile((basePath / filename).str());
    if(!mtlFile.good()) {
        throw std::runtime_error("Cannot open mtl file: " + filename);
    }

    LOG_DEBUG << "Reading material file: " << filename;

    // Read lines into strings
    std::string currentMaterialName;
    std::string lineString;
    std::string commandString;
    std::string parameterString;
    while(std::getline(mtlFile, lineString)) {
        // Strip windows line endings
        stripWindowsLineEnding(lineString);

        // Entirely skip comment or empty lines
        if(lineString[0] == '#' || lineString.empty()) {
            continue;
        }

        // Wrap line in stream for easier parsing
        std::istringstream lineStream(lineString);

        // Read command from first token
        lineStream >> commandString;
        if(commandString == "newmtl") {
            lineStream >> currentMaterialName;
            LOG_INFO << "\tReading material: " << currentMaterialName;
        }
        else if(commandString == "Ns" || commandString == "Ka" || commandString == "Kd"
            || commandString == "Ks" || commandString == "Ke" || commandString == "Ni"
            || commandString == "d" || commandString == "illum")
        {
            // ignore lighting properties
        }
        else if(commandString == "map_Kd") {
            BOB_ASSERT(!currentMaterialName.empty());

            // Skip any whitespace preceeding texture filename
            while(lineStream.peek() == ' ') {
                lineStream.get();
            }

            // Treat remainder of line as texture filename
            std::string textureFilename;
            std::getline(lineStream, textureFilename);
            const size_t firstNonQuote = textureFilename.find_first_not_of('"');
            const size_t lastNonQuote = textureFilename.find_last_not_of('"');
            textureFilename = textureFilename.substr(firstNonQuote, lastNonQuote - firstNonQuote + 1);

            LOG_DEBUG << "\t\tTexture: '" << textureFilename << "'";


            // Load texture and add to map
            const std::string texturePath = (basePath / textureFilename).str();

            // Load texture
            // **NOTE** using OpenCV so as to reduce need for extra dependencies
            cv::Mat texture = cv::imread(texturePath);

            // If texture couldn't be loaded, give warning
            if(texture.cols == 0 && texture.rows == 0) {
                LOG_WARNING << "Cannot load texture '" << texturePath << "'";
            }
            // Otherwise
            else {
                LOG_DEBUG << "\t\t\tOriginal dimensions: " << texture.cols << "x" << texture.rows;

                // If texture isn't square, use longest side as size
                int size = texture.cols;
                if(texture.cols != texture.rows) {
                    size = std::max(texture.cols, texture.rows);
                }

                // Clamp size to maximum texture size
                size = std::min(size, maxTextureSize);

                // Perform resize if required
                if(size != texture.cols || size != texture.rows) {
                    LOG_DEBUG << "\t\t\tResizing to: " << size << "x" << size;
                    cv::resize(texture, texture, cv::Size(size, size), 0, 0, cv::INTER_CUBIC);
                }

                // Flip texture about y-axis as the origin of OpenGL texture coordinates
                // is in the bottom-left and obj file's is in the top-left
                cv::flip(texture, texture, 0);

                // Add a new texture to array and upload data to it in selected format
                m_Textures.emplace_back(new Texture());
                m_Textures.back()->upload(texture, textureFormat);

                // Add name to map
                const bool inserted = textureNames.insert(std::make_pair(currentMaterialName, m_Textures.back().get())).second;
                BOB_ASSERT(inserted);
            }
        }
        else {
            LOG_WARNING << "Unhandled mtl tag '" << commandString << "'";
        }

    }
}

//----------------------------------------------------------------------------
// World::Texture
//----------------------------------------------------------------------------
World::Texture::Texture()
{
    glGenTextures(1, &m_Texture);
}
//----------------------------------------------------------------------------
World::Texture::~Texture()
{
    glDeleteTextures(1, &m_Texture);
}
//----------------------------------------------------------------------------
void World::Texture::bind() const
{
    // Bind texture
    glBindTexture(GL_TEXTURE_2D, m_Texture);
}
//----------------------------------------------------------------------------
void World::Texture::unbind() const
{
    // Unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);
}
//----------------------------------------------------------------------------
void World::Texture::upload(const cv::Mat &texture, GLint textureFormat)
{
    // Bind texture
    glBindTexture(GL_TEXTURE_2D, m_Texture);

    // Configure texture filtering
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // Upload texture data and generate mipmaps
    glTexImage2D(GL_TEXTURE_2D, 0, textureFormat, texture.cols, texture.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, texture.data);
    glGenerateMipmap(GL_TEXTURE_2D);
}


//----------------------------------------------------------------------------
// World::Surface
//----------------------------------------------------------------------------
World::Surface::Surface() : m_PositionVBO(0), m_ColourVBO(0), m_TexCoordVBO(0), m_Texture(nullptr)
{
    // Create a vertex array object to bind everything together
    glGenVertexArrays(1, &m_VAO);
}
//----------------------------------------------------------------------------
World::Surface::~Surface()
{
    if(m_PositionVBO != 0) {
        glDeleteBuffers(1, &m_PositionVBO);
    }

    if(m_ColourVBO != 0) {
        glDeleteBuffers(1, &m_ColourVBO);
    }

    if(m_TexCoordVBO != 0) {
        glDeleteBuffers(1, &m_TexCoordVBO);
    }

    glDeleteVertexArrays(1, &m_VAO);
}
//----------------------------------------------------------------------------
void World::Surface::bind() const
{
    // Bind world VAO
    glBindVertexArray(m_VAO);

    // If surface has a texture, bind it
    if(m_Texture != nullptr) {
        glEnable(GL_TEXTURE_2D);
        m_Texture->bind();
    }
    // Otherwise make sure no textures are bound
    else {
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}
//----------------------------------------------------------------------------
void World::Surface::unbind() const
{
    // If surface has a texture, bind it
    if(m_Texture != nullptr) {
        glDisable(GL_TEXTURE_2D);
        m_Texture->unbind();
    }

    // Unbind vertex array
    glBindVertexArray(0);
}
//----------------------------------------------------------------------------
void World::Surface::render() const
{
    glEnable(GL_CULL_FACE);

    // Draw world
    glDrawArrays(GL_TRIANGLES, 0, m_NumVertices);
}
//----------------------------------------------------------------------------
void World::Surface::uploadPositions(const std::vector<GLfloat> &positions)
{
    // Generate position VBO if required
    if(m_PositionVBO == 0) {
        glGenBuffers(1, &m_PositionVBO);
    }

    // Bind positions buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_PositionVBO);

    // Upload positions
    glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(GLfloat), positions.data(), GL_STATIC_DRAW);

    // Set vertex pointer and enable client state in VAO
    glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_VERTEX_ARRAY);

    // Calculate number of vertices from positions
    m_NumVertices = positions.size() / 3;

    // Unbind buffer
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
//----------------------------------------------------------------------------
void World::Surface::uploadColours(const std::vector<GLfloat> &colours)
{
    // Generate colour VBO if required
    if(m_ColourVBO == 0) {
        glGenBuffers(1, &m_ColourVBO);
    }

    // Bind colours buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_ColourVBO);

    // Upload colours
    glBufferData(GL_ARRAY_BUFFER, colours.size() * sizeof(GLfloat), colours.data(), GL_STATIC_DRAW);

    // Set colour pointer and enable client state in VAO
    glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_COLOR_ARRAY);

    // Unbind buffer
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
//----------------------------------------------------------------------------
void World::Surface::uploadTexCoords(const std::vector<GLfloat> &texCoords)
{
    // Generate colour VBO if required
    if(m_TexCoordVBO == 0) {
        glGenBuffers(1, &m_TexCoordVBO);
    }

    // Bind colours buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_TexCoordVBO);

    // Upload colours
    glBufferData(GL_ARRAY_BUFFER, texCoords.size() * sizeof(GLfloat), texCoords.data(), GL_STATIC_DRAW);

    // Set colour pointer and enable client state in VAO
    glTexCoordPointer(2, GL_FLOAT, 0, BUFFER_OFFSET(0));
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    // Unbind buffer
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
}   // namespace AntWorld
}   // namespace BoBRobotics
