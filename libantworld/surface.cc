#include "surface.h"

// Libantworld includes
#include "common.h"
#include "texture.h"

//----------------------------------------------------------------------------
// BoBRobotics::AntWorlds::Surfaces
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
Surface::Surface() 
:   m_PositionVBO(0), m_ColourVBO(0), m_TexCoordVBO(0), m_IBO(0), 
    m_NumVertices(0), m_NumIndices(0), m_Texture(nullptr)
{
    // Create a vertex array object to bind everything together
    glGenVertexArrays(1, &m_VAO);
}
//----------------------------------------------------------------------------
Surface::~Surface()
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
    
    if(m_IBO != 0) {
        glDeleteBuffers(1, &m_IBO);
    }

    glDeleteVertexArrays(1, &m_VAO);
}
//----------------------------------------------------------------------------
void Surface::bind() const
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
void Surface::unbind() const
{
    // If surface has a texture, bind it
    if(m_Texture != nullptr) {
        glDisable(GL_TEXTURE_2D);
        m_Texture->unbind();
    }
    
    if(m_IBO != 0) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    // Unbind vertex array
    glBindVertexArray(0);
}
//----------------------------------------------------------------------------
void Surface::render(GLenum primitive) const
{
    glEnable(GL_CULL_FACE);

    if(m_IBO == 0) {
        // Draw world
        glDrawArrays(primitive, 0, m_NumVertices);
    }
    else {
        // Draw render mesh quads
        glDrawElements(primitive, m_NumIndices, GL_UNSIGNED_INT, BUFFER_OFFSET(0));
    }
}
//----------------------------------------------------------------------------
void Surface::uploadPositions(const std::vector<GLfloat> &positions)
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
void Surface::uploadColours(const std::vector<GLfloat> &colours)
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
void Surface::uploadTexCoords(const std::vector<GLfloat> &texCoords)
{
    // Generate texture coordinates VBO if required
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
//----------------------------------------------------------------------------
void Surface::uploadIndices(const std::vector<GLuint> &indices)
{
    // Generate IBO if required
    if(m_IBO == 0) {
        glGenBuffers(1, &m_IBO);
    }
    
    // Cache number of indices
    m_NumIndices = indices.size();
    
    // Bind and upload index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_IBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);// Unbind buffer
    
    // Unbind buffer
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
}   // namespace AntWorld
}   // namespace BoBRobotics
