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
    Renderer(unsigned int displayRenderWidth, unsigned int displayRenderHeight);
    Renderer(const std::string &worldFilename, const GLfloat (&worldColour)[3], const GLfloat (&groundColour)[3],
             unsigned int displayRenderWidth, unsigned int displayRenderHeight);
    ~Renderer();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool loadWorld(const std::string &filename, const GLfloat (&worldColour)[3],
                   const GLfloat (&groundColour)[3]);
    bool loadWorldObj(const std::string &objFilename, int maxTextureSize = -1, GLint textureFormat = GL_RGB);

    void renderAntView(float antX, float antY, float antHeading);
    void renderTopDownView();
    void render(float antX, float antY, float antHeading);

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

    const unsigned int m_DisplayRenderWidth;
    const unsigned int m_DisplayRenderHeight;
};
}   // namespace AntWorld
}   // namespace BoBRobotics