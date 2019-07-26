#pragma once

// Standard C++ includes
#include <vector>

// OpenGL includes
#include <GL/glew.h>

// Libantworld includes
#include "antworld/common.h"
#include "antworld/opengl_type_traits.h"

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
    void bind() const;
    void unbind() const;

    void bindTextured() const;
    void unbindTextured() const;

    void unbindIndices() const;
    void render(GLenum primitive = GL_TRIANGLES, GLenum indexType = GL_UNSIGNED_INT) const;

    template<typename T>
    void uploadPositions(const std::vector<T> &positions, GLint size = 3)
    {
        // Upload positions to buffer
        uploadBuffer(positions, m_PositionVBO, GL_ARRAY_BUFFER, GL_STATIC_DRAW);

        // Set vertex pointer and enable client state in VAO
        glVertexPointer(size, OpenGLTypeTraits<T>::type, 0, BUFFER_OFFSET(0));
        glEnableClientState(GL_VERTEX_ARRAY);

        // Calculate number of vertices from positions
        m_NumVertices = positions.size() / size;

        // Unbind buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    template<typename T>
    void uploadColours(const std::vector<T> &colours, GLint size = 3)
    {
        // Upload colours to buffer
        uploadBuffer(colours, m_ColourVBO, GL_ARRAY_BUFFER, GL_STATIC_DRAW);

        // Set colour pointer and enable client state in VAO
        glColorPointer(size, OpenGLTypeTraits<T>::type, 0, BUFFER_OFFSET(0));
        glEnableClientState(GL_COLOR_ARRAY);

        // Unbind buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    template<typename T>
    void uploadTexCoords(const std::vector<T> &texCoords, GLint size = 2)
    {
        // Upload texture coordinates to buffer
        uploadBuffer(texCoords, m_TexCoordVBO, GL_ARRAY_BUFFER, GL_STATIC_DRAW);

        // Set colour pointer and enable client state in VAO
        glTexCoordPointer(size, OpenGLTypeTraits<T>::type, 0, BUFFER_OFFSET(0));
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);

        // Unbind buffer
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    template<typename T>
    void uploadIndices(const std::vector<T> &indices)
    {
        // Upload indices
        uploadBuffer(indices, m_IBO, GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW);

        // Cache number of indices
        m_NumIndices = indices.size();

        // **NOTE** GL_ELEMENT_ARRAY_BUFFER works subtly different from GL_ARRAY_BUFFER
        // as it has no client state/pointer tying it to the VAO. Therefore we need to
        // leave it bound until after we unbind the VAO - makes sense but ugly
    }
    
    void setTexture(const Texture *texture){ m_Texture = texture; }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    template<typename T>
    void uploadBuffer(const std::vector<T> &data, GLuint &bufferObject,
                      GLenum target, GLenum usage)
    {
        // Generate buffer if required
        if(bufferObject == 0) {
            glGenBuffers(1, &bufferObject);
        }

        // Bind buffer
        glBindBuffer(target, bufferObject);

        // Upload data
        glBufferData(target, data.size() * sizeof(T), data.data(), usage);
    }

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
