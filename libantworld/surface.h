#pragma once

// Standard C++ includes
#include <vector>

// OpenGL includes
#include <GL/glew.h>

// Forward declarations
namespace BoBRobotics
{
namespace AntWorld
{
class Texture;
}
}

//------------------------------------------------------------------------
// Surface
//------------------------------------------------------------------------
// Encapsulates a single 'surface' - geometry to be rendered in one draw call using one material
namespace BoBRobotics
{
namespace AntWorld
{
class Surface
{
public:
    Surface();
    ~Surface();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void bind(bool bindTexture = true) const;
    void unbind(bool unbindTexture = true) const;
    void render(GLenum primitive = GL_TRIANGLES) const;

    void uploadPositions(const std::vector<GLfloat> &positions);
    void uploadColours(const std::vector<GLfloat> &positions);
    void uploadTexCoords(const std::vector<GLfloat> &texCoords);
    void uploadIndices(const std::vector<GLuint> &indices);
    
    void setTexture(const Texture *texture){ m_Texture = texture; }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    GLuint m_VAO;
    GLuint m_PositionVBO;
    GLuint m_ColourVBO;
    GLuint m_TexCoordVBO;
    GLuint m_IBO;
    
    unsigned int m_NumVertices;
    unsigned int m_NumIndices;

    const Texture *m_Texture;
};
}   // namespace AntWorld
}   // namespace BoBRobotics