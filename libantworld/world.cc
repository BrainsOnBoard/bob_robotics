#include "world.h"

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <tuple>

// Standard C includes
#include <cassert>

// OpenCV includes
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "../third_party/path.h"

// Antworld includes
#include "common.h"

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
template<unsigned int N>
void readVector(std::istringstream &stream, std::vector<GLfloat> &vector)
{
    // Ensure there is space for vector
    vector.reserve(vector.size() + N);

    // Read components and push back
    GLfloat x;
    for(unsigned int i = 0; i < N; i++) {
        stream >> x;
        vector.push_back(x);
    }

    // Check this is the end of the linestream i.e. there aren't extra components
    assert(stream.eof());
}

void readFace(std::istringstream &lineStream,
              const std::vector<GLfloat> &rawPositions,
              const std::vector<GLfloat> &rawTexCoords,
              std::tuple<std::string, std::vector<GLfloat>, std::vector<GLfloat>> &currentObjSurface)
{
     // Get references to current material's positions and texture coordinates
    auto &surfacePositions = std::get<1>(currentObjSurface);
    auto &surfaceTexCoords = std::get<2>(currentObjSurface);

    // Reserve memory for triangle's vertex positions and texture coordinates
    surfacePositions.reserve(surfacePositions.size() + (3 * 3));
    surfaceTexCoords.reserve(surfaceTexCoords.size() + (3 * 2));

    // Loop through face vertex
    std::string faceIndexString;
    std::string indexString;
    for(unsigned int v = 0; v < 3; v++) {
        // Read indices i.e. P/T/N into string
        lineStream >> faceIndexString;

        // Convert into stream for processing
        std::istringstream faceIndexStream(faceIndexString);

        // Extract index of raw position and texture coordinate
        std::getline(faceIndexStream, indexString, '/');
        const int position = stoi(indexString);
        std::getline(faceIndexStream, indexString, '/');
        const int texCoord = stoi(indexString);

        // Copy raw positions and texture coordinates into material
        std::copy_n(&rawPositions[3 * position], 3, std::back_inserter(surfacePositions));
        std::copy_n(&rawTexCoords[2 * texCoord], 2, std::back_inserter(surfaceTexCoords));
    }

    // Check this is the end of the linestream i.e. there aren't extra components
    assert(lineStream.eof());
}

bool parseMaterials(const filesystem::path &basePath, const std::string &filename, std::map<std::string, cv::Mat> &textures)
{
    // Open obj file
    std::ifstream mtlFile((basePath / filename).str());
    if(!mtlFile.good()) {
        std::cerr << "Cannot open mtl file: " << filename << std::endl;
        return false;
    }

    // Read lines into strings
    std::string currentMaterialName;
    std::string lineString;
    std::string commandString;
    std::string parameterString;
    while(std::getline(mtlFile, lineString)) {
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
            std::cout << "Beginning material: " << currentMaterialName << std::endl;
        }
        else if(commandString == "Ns" || commandString == "Ka" || commandString == "Kd"
            || commandString == "Ks" || commandString == "Ke" || commandString == "Ni"
            || commandString == "d" || commandString == "illum")
        {
            // ignore lighting properties
        }
        else if(commandString == "map_Kd") {
            assert(!currentMaterialName.empty());

            const std::string textureFilename = lineStream.str();
            std::cout << "Texture: " << textureFilename << std::endl;

            // Load texture and add to map
            // **NOTE** using OpenCV so as to reduce need for extra dependencies
            textures.emplace(currentMaterialName, cv::imread(textureFilename));
        }
        else {
            std::cerr << "WARNING: unhandled mtl tag '" << commandString << "'" << std::endl;
        }

    }

    return true;
}
}

//----------------------------------------------------------------------------
// World
//----------------------------------------------------------------------------
bool World::load(const std::string &filename, const GLfloat (&worldColour)[3],
                 const GLfloat (&groundColour)[3])
{
    // Create single surface
    m_Surfaces.clear();
    m_Surfaces.emplace_back();
    auto &surface = m_Surfaces.back();

    // Open file for binary IO
    std::ifstream input(filename, std::ios::binary);
    if(!input.good()) {
        std::cerr << "Cannot open world file:" << filename << std::endl;
        return false;
    }

    // Seek to end of file, get size and rewind
    input.seekg(0, std::ios_base::end);
    const std::streampos numTriangles = input.tellg() / (sizeof(double) * 12);
    input.seekg(0);
    std::cout << "World has " << numTriangles << " triangles" << std::endl;

    // Bind surface vertex array
    surface.bind();

    {
        // Reserve 3 XYZ positions for each triangle and 6 for the ground
        std::vector<GLfloat> positions((6 + (numTriangles * 3)) * 3);

        // Add first ground triangle vertex positions
        positions[0] = 0.0f;    positions[1] = 0.0f;    positions[2] = 0.0f;
        positions[3] = 10.5f;   positions[4] = 10.5f;   positions[5] = 0.0f;
        positions[6] = 0.0f;    positions[7] = 10.5f;   positions[8] = 0.0f;

        // Add second ground triangle vertex positions
        positions[9] = 0.0f;    positions[10] = 0.0f;   positions[11] = 0.0f;
        positions[12] = 10.5f;  positions[13] = 0.0f;   positions[14] = 0.0f;
        positions[15] = 10.5f;  positions[16] = 10.5f;  positions[17] = 0.0f;

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
                    positions[18 + (t * 9) + (v * 3) + c] = (GLfloat)trianglePosition;
                }
            }
        }

        // Upload positions
        surface.uploadPositions(positions);
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

    return true;
}
//----------------------------------------------------------------------------
bool World::loadObj(const std::string &filename)
{
    // Vector of geometry associated with each named surface (in unindexed triangle format)
    std::vector<std::tuple<std::string, std::vector<GLfloat>, std::vector<GLfloat>>> objSurfaces;

    std::map<std::string, cv::Mat> textures;

    // Parser
    {
        // Vectors to hold 'raw' positions and texture coordinates read from obj
        std::vector<GLfloat> rawPositions;
        std::vector<GLfloat> rawTexCoords;

        // Open obj file
        std::ifstream objFile(filename);
        if(!objFile.good()) {
            std::cerr << "Cannot open obj file: " << filename << std::endl;
            return false;
        }

        // Read lines into strings
        std::string lineString;
        std::string commandString;
        std::string parameterString;
        while(std::getline(objFile, lineString)) {
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
                std::cout << "Using material file: " << parameterString << std::endl;

                const auto objPath = filesystem::path(filename).make_absolute();
                parseMaterials(objPath.parent_path(), parameterString, textures);
            }
            else if(commandString == "o") {
                lineStream >> parameterString;
                std::cout << "Reading object: " << parameterString << std::endl;
            }
            else if(commandString == "v") {
                // Read vertex
                readVector<3>(lineStream, rawPositions);
            }
            else if(commandString == "vt") {
                // Read texture coordinate
                readVector<2>(lineStream, rawTexCoords);
            }
            else if(commandString == "vn") {
                // ignore vertex normals for now
            }
            else if(commandString == "usemtl") {
                lineStream >> parameterString;
                std::cout << "Beginning surface: " << parameterString << std::endl;
                objSurfaces.emplace_back(parameterString, std::initializer_list<GLfloat>(), std::initializer_list<GLfloat>());
            }
            else if(commandString == "s") {
                // ignore smoothing
            }
            else if(commandString == "f") {
                // Check that a surface has been begun
                assert(!objSurfaces.empty());

                // Read face
                readFace(lineStream, rawPositions, rawTexCoords, objSurfaces.back());
            }
            else {
                std::cerr << "WARNING: unhandled obj tag '" << commandString << "'" << std::endl;
            }

        }

        std::cout << rawPositions.size() / 3 << " raw positions, " << rawTexCoords.size() / 2 << " raw texture coordinates, ";
        std::cout << objSurfaces.size() << " surfaces, " << textures.size() << " textures" << std::endl;
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
        const auto tex = textures.find(std::get<0>(objSurface));
        if(tex != textures.end()) {
            surface.uploadTexture(tex->second);
        }
    }


    return true;
}
//----------------------------------------------------------------------------
void World::render() const
{
    // Bind and render each material
    for(auto &surf : m_Surfaces) {
        surf.bind();
        surf.render();
    }
}

//----------------------------------------------------------------------------
// World::Surface
//----------------------------------------------------------------------------
World::Surface::Surface() : m_PositionVBO(0), m_ColourVBO(0), m_TexCoordVBO(0), m_Texture(0)
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

    if(m_Texture != 0) {
        glDeleteTextures(1, &m_Texture);
    }

    glDeleteVertexArrays(1, &m_VAO);
}
//----------------------------------------------------------------------------
void World::Surface::bind() const
{
    // Bind world VAO
    glBindVertexArray(m_VAO);

    // Bind texture
    glBindTexture(GL_TEXTURE_2D, m_Texture);
}
//----------------------------------------------------------------------------
void World::Surface::render() const
{
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
}
//----------------------------------------------------------------------------
void World::Surface::uploadTexture(const cv::Mat &texture)
{
    // Generate texture object if required
    if(m_Texture == 0) {
        glGenTextures(1, &m_Texture);
    }

    // Bind texture
    glBindTexture(GL_TEXTURE_2D, m_Texture);

    // Upload texture data and generate mipmaps
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB8, texture.cols, texture.rows, GL_BGR, GL_UNSIGNED_BYTE, texture.data);
}