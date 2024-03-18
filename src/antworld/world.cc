// BoB robotics includes
#include "antworld/common.h"
#include "antworld/world.h"
#include "common/macros.h"
#include "plog/Log.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <tuple>

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
template<unsigned int N, typename T, typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr>
void readVector(std::istringstream &stream, std::vector<T> &vector, float scale)
{
    // Read components and push back
    GLfloat x;
    for(unsigned int i = 0; i < N; i++) {
        stream >> x;
        vector.push_back(x * scale);
    }
}
//----------------------------------------------------------------------------
template<unsigned int N, typename T, typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
void readVector(std::istringstream &stream, std::vector<T> &vector, float scale)
{
    // Read components and push back
    GLfloat x;
    for(unsigned int i = 0; i < N; i++) {
        stream >> x;
        vector.push_back((T)std::round(x * scale));
    }
}
//----------------------------------------------------------------------------
void readFace(std::istringstream &lineStream,
              const std::vector<GLfloat> &rawPositions,
              const std::vector<GLbyte> &rawColours,
              const std::vector<GLfloat> &rawTexCoords,
              std::tuple<std::string, std::vector<GLfloat>, std::vector<GLbyte>, std::vector<GLfloat>> &currentObjSurface)
{
    // Get references to current material's positions and texture coordinates
    auto &surfacePositions = std::get<1>(currentObjSurface);
    auto &surfaceColours = std::get<2>(currentObjSurface);
    auto &surfaceTexCoords = std::get<3>(currentObjSurface);

    // Loop through face vertex
    std::string faceIndexString;
    std::string indexString;
    for(unsigned int v = 0; v < 3; v++) {
        // Read indices i.e. P/T/N into string
        lineStream >> faceIndexString;

        // Convert into stream for processing
        std::istringstream faceIndexStream(faceIndexString);

        // Extract index of raw position
        std::getline(faceIndexStream, indexString, '/');
        const int position = stoi(indexString) - 1;

        // Copy raw position into material
        std::copy_n(&rawPositions[3 * position], 3, std::back_inserter(surfacePositions));

        // If there are any colours, copy them into material
        if(!rawColours.empty()) {
            std::copy_n(&rawColours[3 * position], 3, std::back_inserter(surfaceColours));
        }

        // If there is more data to read
        if(!faceIndexStream.eof()) {
            // Extract index of raw texture coordinate
            std::getline(faceIndexStream, indexString, '/');
            const int texCoord = stoi(indexString) - 1;

            // Copy raw texture coordinates into material
            std::copy_n(&rawTexCoords[2 * texCoord], 2, std::back_inserter(surfaceTexCoords));
        }
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
//----------------------------------------------------------------------------
std::string readName(std::istringstream &lineStream)
{
    // Skip any whitespace preceeding name
    while(lineStream.peek() == ' ') {
        lineStream.get();
    }

    // Treat remainder of line as name
    std::string name;
    std::getline(lineStream, name);
    const size_t firstNonQuote = name.find_first_not_of('"');
    const size_t lastNonQuote = name.find_last_not_of('"');
    return name.substr(firstNonQuote, lastNonQuote - firstNonQuote + 1);
}
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::World
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
void World::load(const filesystem::path &filename, const GLfloat (&worldColour)[3],
                 const GLfloat (&groundColour)[3], bool clear)
{
    LOGI << "Loading " << filename << "...";

    // Clear existing surfaces if required
    if(clear) {
        m_Surfaces.clear();
    }

    // Create single surface
    m_Surfaces.emplace_back();
    auto &surface = m_Surfaces.back();

    // Open file for binary IO
    std::ifstream input(filename.str(), std::ios::binary);
    input.exceptions(std::ios::badbit | std::ios::failbit);

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
        positions[0] = static_cast<float>(m_MinBound[0].value());
        positions[1] = static_cast<float>(m_MinBound[1].value());
        positions[2] = 0.0f;
        positions[3] = static_cast<float>(m_MaxBound[0].value());
        positions[4] = static_cast<float>(m_MaxBound[1].value());
        positions[5] = 0.0f;
        positions[6] = static_cast<float>(m_MinBound[0].value());
        positions[7] = static_cast<float>(m_MaxBound[1].value());
        positions[8] = 0.0f;

        // Add second ground plane triangle vertex positions
        positions[9] = static_cast<float>(m_MinBound[0].value());
        positions[10] = static_cast<float>(m_MinBound[1].value());
        positions[11] = 0.0f;
        positions[12] = static_cast<float>(m_MaxBound[0].value());
        positions[13] = static_cast<float>(m_MinBound[1].value());
        positions[14] = 0.0f;
        positions[15] = static_cast<float>(m_MaxBound[0].value());
        positions[16] = static_cast<float>(m_MaxBound[1].value());
        positions[17] = 0.0f;

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
                colours[18 + (t * 9) + (v * 3)] = worldColour[0] * static_cast<float>(triangleColour);
                colours[18 + (t * 9) + (v * 3) + 1] = worldColour[1] * static_cast<float>(triangleColour);
                colours[18 + (t * 9) + (v * 3) + 2] = worldColour[2] * static_cast<float>(triangleColour);
            }
        }

        // Upload colours
        surface.uploadColours(colours);
    }

    // Unbind surface
    surface.unbind();
}
//----------------------------------------------------------------------------
void World::loadObj(const filesystem::path &filename, float scale,
                    int maxTextureSize, GLint textureFormat, bool clear)
{
    LOGI << "Loading " << filename << "...";

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
    // Each surface consists of:
    // 0 - a name
    // 1 - a vector of floating point vertex positions (x, y, z)
    // 2 - a vector of 8-bit colours (r, g, b)
    // 3 - a vector of floating point texture coordinates (U, V)
    std::vector<std::tuple<std::string, std::vector<GLfloat>, std::vector<GLbyte>, std::vector<GLfloat>>> objSurfaces;

    // Map of material names to texture pointers and colour names
    std::map<std::string, std::tuple<Texture*, Surface::Colour>> materialNames;

    // Parser
    {
        // Vectors to hold 'raw' positions, colours and texture coordinates read from obj
        std::vector<GLfloat> rawPositions;
        std::vector<GLbyte> rawColours;
        std::vector<GLfloat> rawTexCoords;

        // Open obj file
        std::ifstream objFile(filename.str());
        BOB_ASSERT(!objFile.fail());
        objFile.exceptions(std::ios::badbit);

        // Get base path to load materials etc relative to
        const auto basePath = filename.make_absolute().parent_path();

        // Read lines into strings
        std::string lineString;
        std::string commandString;
        std::string parameterString;
        while(std::getline(objFile, lineString)) {
            // Strip windows line endings
            stripWindowsLineEnding(lineString);

            // Entirely skip comment or empty lines
            if(lineString.empty() || lineString[0] == '#') {
                continue;
            }

            // Wrap line in stream for easier parsing
            std::istringstream lineStream(lineString);

            // Read command from first token
            lineStream >> commandString;
            if(commandString == "mtllib") {
                // Parse materials
                loadMaterials(basePath, readName(lineStream),
                              textureFormat, maxTextureSize,
                              materialNames);
            }
            else if(commandString == "o") {
                lineStream >> parameterString;
                LOG_DEBUG << "Reading object: " << parameterString;
            }
            else if(commandString == "v") {
                // Read vertex
                readVector<3>(lineStream, rawPositions, scale);

                // If line has more data, read it into raw colours
                if(!lineStream.eof()) {
                    readVector<3>(lineStream, rawColours, 255.0f);
                }
            }
            else if(commandString == "vt") {
                // Read texture coordinate and check there's no unhandled components following it
                readVector<2>(lineStream, rawTexCoords, 1.0f);
                BOB_ASSERT(lineStream.eof());
            }
            else if(commandString == "vn") {
                // ignore vertex normals for now
            }
            else if(commandString == "usemtl") {
                lineStream >> parameterString;
                LOG_INFO << "\tReading surface: " << parameterString;
                objSurfaces.emplace_back(parameterString, std::initializer_list<GLfloat>(),
                                         std::initializer_list<GLbyte>(), std::initializer_list<GLfloat>());
            }
            else if(commandString == "s") {
                // ignore smoothing
            }
            else if(commandString == "f") {
                // If there are no textures, surfaces aren't always created (at least by MeshLab), so create a default one
                if(objSurfaces.empty()) {
                    LOG_WARNING << "Encountered faces before any surfaces are defined - adding default surface";
                    objSurfaces.emplace_back("default", std::initializer_list<GLfloat>(),
                                             std::initializer_list<GLbyte>(), std::initializer_list<GLfloat>());
                }

                // Read face
                readFace(lineStream, rawPositions, rawColours, rawTexCoords, objSurfaces.back());
            }
            else {
                LOG_WARNING << "Unhandled obj tag '" << commandString << "'";
            }

        }

        LOG_INFO << "\t" << rawPositions.size() / 3 << " raw positions, " << rawTexCoords.size() / 2 << " raw texture coordinates, ";
        LOG_INFO << rawColours.size() / 3 << " raw colours, " << objSurfaces.size() << " surfaces, " << m_Textures.size() << " textures";

        // If there are ANY raw colours, assert that there are the same number as there are positions
        if(!rawColours.empty()) {
            BOB_ASSERT(rawColours.size() == rawPositions.size());
        }

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
    if(clear) {
        m_Surfaces.clear();
    }
    
    // Allocate new materials array to match those found in obj
    m_Surfaces.reserve(m_Surfaces.size() + objSurfaces.size());

    // Loop through surfaces
    for(const auto &objSurface : objSurfaces) {
        // Create surface
        m_Surfaces.emplace_back();
        auto &surface = m_Surfaces.back();
        
        // Find corresponding material
        const auto mtl = materialNames.find(std::get<0>(objSurface));

        // Bind material
        surface.bind();

        // Upload positions from obj file
        surface.uploadPositions(std::get<1>(objSurface));

        // If material was found, set colour
        if(mtl != materialNames.end()) {
            surface.setColour(std::get<1>(mtl->second));
        }

        // If there are any vertex colours, upload them from obj file
        if(!std::get<2>(objSurface).empty()) {
            surface.uploadColours(std::get<2>(objSurface));
        }

        // If there are any texture coordinates
        if(!std::get<3>(objSurface).empty()) {
            // Upload texture coordinates from obj file
            surface.uploadTexCoords(std::get<3>(objSurface));

            // If material was found, set texture
            if(mtl != materialNames.end()) {
                surface.setTexture(std::get<0>(mtl->second));
            }
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
        surf.bindTextured();
        surf.render();
        surf.unbindTextured();
    }
}
//----------------------------------------------------------------------------
void World::loadMaterials(const filesystem::path &basePath, const std::string &filename,
                          GLint textureFormat, int maxTextureSize,
                          std::map<std::string, std::tuple<Texture*, Surface::Colour>> &materialNames)
{
    // Open obj file
    std::ifstream mtlFile((basePath / filename).str());
    BOB_ASSERT(!mtlFile.fail());
    mtlFile.exceptions(std::ios::badbit);

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

        // If command is a material name
        if(commandString == "newmtl") {
            lineStream >> currentMaterialName;
            LOG_INFO << "\tReading material: " << currentMaterialName;
        }
        // Otherwise, if command specifies a diffuse colour
        else if(commandString == "Kd") {
            // Read colour
            Surface::Colour colour;
            for(unsigned int i = 0; i < 3; i++) {
                lineStream >> colour[i];
            }

            // If material doesn't yet exist, emplace new material with colour and no texture
            auto mtl = materialNames.find(currentMaterialName);
            if(mtl == materialNames.cend()) {
                materialNames.emplace(std::piecewise_construct,
                                      std::forward_as_tuple(currentMaterialName),
                                      std::forward_as_tuple(nullptr, colour));
            }
            // Otherwise, set colour in existing material
            else {
                std::get<1>(mtl->second) = colour;
            }
        }
        // Otherwise, if command specifies a diffuse map
        else if(commandString == "map_Kd") {
            BOB_ASSERT(!currentMaterialName.empty());


            std::string textureFilename = readName(lineStream);

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

                // If material doesn't yet exist, emplace new material with texture and default colour
                auto mtl = materialNames.find(currentMaterialName);
                if(mtl == materialNames.cend()) {
                    materialNames.emplace(std::piecewise_construct,
                                          std::forward_as_tuple(currentMaterialName),
                                          std::forward_as_tuple(m_Textures.back().get(), Surface::DefaultColour));
                }
                // Otherwise
                else {
                    // Ensure texture isn't already set for this material
                    BOB_ASSERT(std::get<0>(mtl->second) == nullptr);

                    // Set texture in material
                    std::get<0>(mtl->second) = m_Textures.back().get();
                }
            }
        }
        // Otherwise, if it's another material property, ignore
        else if(commandString == "Ns" || commandString == "Ka" || commandString == "Ks" ||
                commandString == "Ke" || commandString == "Ni" || commandString == "d" ||
                commandString == "illum")
        {
        }
        // Otherwise, give warning
        else {
            LOG_WARNING << "Unhandled mtl tag '" << commandString << "'";
        }

    }
}
}   // namespace AntWorld
}   // namespace BoBRobotics
