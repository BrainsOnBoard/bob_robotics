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
    Renderer(unsigned int cubemapSize = 256);
    ~Renderer();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void renderAntView(float antX, float antY, float antHeading,
                       GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight);
    void renderTopDownView(GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight);

    World &getWorld(){ return m_World; }
    const World &getWorld() const{ return m_World; }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    void generateCubeFaceLookAtMatrices();

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
};
}   // namespace AntWorld
}   // namespace BoBRobotics