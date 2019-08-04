#include "antworld/surface.h"
#include "antworld/texture.h"

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
}
//----------------------------------------------------------------------------
void Surface::unbind() const
{
    // Unbind vertex array
    glBindVertexArray(0);
}
//----------------------------------------------------------------------------
void Surface::bindTextured() const
{
    bind();

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
void Surface::unbindTextured() const
{
    unbind();

    // If surface has a texture, unbind it
    if(m_Texture != nullptr) {
        glDisable(GL_TEXTURE_2D);
        m_Texture->unbind();
    }
}
//----------------------------------------------------------------------------
void Surface::unbindIndices() const
{
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
//----------------------------------------------------------------------------
void Surface::render(GLenum primitive, GLenum indexType) const
{
    glEnable(GL_CULL_FACE);

    if(m_IBO == 0) {
        // Draw world
        glDrawArrays(primitive, 0, m_NumVertices);
    }
    else {
        // Draw render mesh quads
        glDrawElements(primitive, m_NumIndices, indexType, BUFFER_OFFSET(0));
    }
}

}   // namespace AntWorld
}   // namespace BoBRobotics
