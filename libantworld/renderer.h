#pragma once

// Standard C++ includes
#include <string>

// OpenGL includes
#include <GL/glew.h>

// Antworld includes
#include "render_mesh.h"
#include "world.h"

//----------------------------------------------------------------------------
// Renderer
//----------------------------------------------------------------------------
// Helper class which combines a world with a rendermesh to allow ant views of world to be rendered to screen
namespace BoBRobotics
{
namespace AntWorld
{
class Renderer
{
public:
    Renderer(unsigned int cubemapSize = 256, double nearClip = 0.001, double farClip = 1000.0);
    ~Renderer();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void renderPanoramicView(float x, float y, float z,
                             float yaw, float pitch, float roll,
                             GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight);
    void renderFirstPersonView(float x, float y, float z,
                               float yaw, float pitch, float roll,
                               GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight);
    void renderTopDownView(GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight);

    World &getWorld(){ return m_World; }
    const World &getWorld() const{ return m_World; }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    void generateCubeFaceLookAtMatrices();
    void applyFrame(float x, float y, float z,
                    float yaw, float pitch, float roll);

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    World m_World;
    RenderMesh m_RenderMesh;

    GLuint m_CubemapTexture;
    GLuint m_FBO;
    GLuint m_DepthBuffer;
    GLfloat m_CubeFaceLookAtMatrices[6][16];

    const unsigned int m_CubemapSize;
    const double m_NearClip;
    const double m_FarClip;
};
}   // namespace AntWorld
}   // namespace BoBRobotics