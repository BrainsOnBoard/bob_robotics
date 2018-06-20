#include "renderer.h"

// Standard C++ includes
#include <stdexcept>

//------------------------------------------------------------------------
// BoBRobotics::AntWorld::Renderer
//------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
// **NOTE** RenderMesh initialisation matches the matlab:
// hfov = hfov/180/2*pi;
// axis([0 14 -hfov hfov -pi/12 pi/3]);
Renderer::Renderer(unsigned int displayRenderWidth, unsigned int displayRenderHeight)
:   m_RenderMesh(296.0f, 75.0f, 15.0f, 40, 10),
    m_CubemapTexture(0), m_FBO(0), m_DepthBuffer(0),
    m_DisplayRenderWidth(displayRenderWidth), m_DisplayRenderHeight(displayRenderHeight)
{
     // Create FBO for rendering to cubemap and bind
    glGenFramebuffers(1, &m_FBO);
    glBindFramebuffer(GL_FRAMEBUFFER, m_FBO);

    // Create cubemap and bind
    glGenTextures(1, &m_CubemapTexture);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_CubemapTexture);

    // Create textures for all faces of cubemap
    // **NOTE** even though we don't need top and bottom faces we still need to create them or rendering fails
    for(unsigned int t = 0; t < 6; t++) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + t, 0, GL_RGB,
                     256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
    }
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    // Create depth render buffer
    glGenRenderbuffers(1, &m_DepthBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, m_DepthBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, 256, 256);

    // Attach depth buffer to frame buffer
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_DepthBuffer);

    // Check frame buffer is created correctly
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error("Frame buffer not complete");
    }

    // Unbind cube map and frame buffer
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Pre-generate lookat matrices to point at cubemap faces
    generateCubeFaceLookAtMatrices();
}
//----------------------------------------------------------------------------
Renderer::Renderer(const std::string &worldFilename, const GLfloat (&worldColour)[3], const GLfloat (&groundColour)[3],
                   unsigned int displayRenderWidth, unsigned int displayRenderHeight)
:   Renderer(displayRenderWidth, displayRenderHeight)
{
    if(!loadWorld(worldFilename, worldColour, groundColour)){
        throw std::runtime_error("Cannot load world");
    }
}
//----------------------------------------------------------------------------
Renderer::~Renderer()
{
    glDeleteRenderbuffers(1, &m_DepthBuffer);
    glDeleteTextures(1, &m_CubemapTexture);
    glDeleteFramebuffers(1, &m_FBO);
}
//----------------------------------------------------------------------------
bool Renderer::loadWorld(const std::string &filename, const GLfloat (&worldColour)[3],
                    const GLfloat (&groundColour)[3])
{
    return m_World.load(filename, worldColour, groundColour);
}
//----------------------------------------------------------------------------
bool Renderer::loadWorldObj(const std::string &objFilename)
{
    return m_World.loadObj(objFilename);
}
//----------------------------------------------------------------------------
void Renderer::renderAntView(float antX, float antY, float antHeading)
{
    // Configure viewport to cubemap-sized square
    glViewport(0, 0, 256, 256);

    // Bind the cubemap FBO for offscreen rendering
    glBindFramebuffer(GL_FRAMEBUFFER, m_FBO);

    // Configure perspective projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90.0,
                   1.0,
                   0.001, 14.0);

    glMatrixMode(GL_MODELVIEW);

    // Save ant transform to matrix
    float antMatrix[16];
    glLoadIdentity();
    glRotatef(antHeading, 0.0f, 0.0f, 1.0f);
    glTranslatef(-antX, -antY, -0.01f);
    glGetFloatv(GL_MODELVIEW_MATRIX, antMatrix);

    // Loop through each heading we need to render
    for(GLenum f = 0; f < 6; f++) {
        // Attach correct frame buffer face to frame buffer
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, f + GL_TEXTURE_CUBE_MAP_POSITIVE_X, m_CubemapTexture, 0);

        // Load look at matrix for this cube face
        glLoadMatrixf(m_CubeFaceLookAtMatrices[f]);

        // Multiply this by ant transform
        glMultMatrixf(antMatrix);

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw world
        m_World.render();
    }

    // Unbind the FBO for onscreen rendering
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Set viewport to strip at stop of window
    glViewport(0, m_DisplayRenderWidth + 10,
               m_DisplayRenderWidth, m_DisplayRenderHeight);

    // Bind cubemap texture
    glEnable(GL_TEXTURE_CUBE_MAP);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_CubemapTexture);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0,
               0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Render render mesh
    m_RenderMesh.render();

    // Disable texture coordinate array, cube map texture and cube map texturing!
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    glDisable(GL_TEXTURE_CUBE_MAP);
}
//----------------------------------------------------------------------------
void Renderer::renderTopDownView()
{
    // Set viewport to square at bottom of screen
    glViewport(0, 0, m_DisplayRenderWidth, m_DisplayRenderWidth);

    // Configure top-down orthographic projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, 10.0,
               0.0, 10.0);

    // Build modelview matrix to centre world
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Render world
    m_World.render();
}
//----------------------------------------------------------------------------
void Renderer::render(float antX, float antY, float antHeading)
{
    // Render ant's eye view at top of the screen
    renderAntView(antX, antY, antHeading);

    // Render top-down view at bottom of the screen
    renderTopDownView();
}
//----------------------------------------------------------------------------
void Renderer::generateCubeFaceLookAtMatrices()
{
    // Set matrix model (which matrix stack you trash is somewhat arbitrary)
    glMatrixMode(GL_MODELVIEW);

    // Loop through cube faces
    for(unsigned int f = 0; f < 6; f++) {
        // Load identity matrix
        glLoadIdentity();

        // Load lookup matrix
        switch (f + GL_TEXTURE_CUBE_MAP_POSITIVE_X)
        {
            case GL_TEXTURE_CUBE_MAP_POSITIVE_X:
                gluLookAt(0.0,  0.0,    0.0,
                          1.0,  0.0,    0.0,
                          0.0,  0.0,    1.0);
                break;

            case GL_TEXTURE_CUBE_MAP_NEGATIVE_X:
                gluLookAt(0.0,  0.0,    0.0,
                          -1.0, 0.0,    0.0,
                          0.0,  0.0,    1.0);
                break;

            case GL_TEXTURE_CUBE_MAP_POSITIVE_Y:
                gluLookAt(0.0,  0.0,    0.0,
                          0.0,  0.0,    -1.0,
                          0.0,  1.0,    0.0);
                break;

            case GL_TEXTURE_CUBE_MAP_NEGATIVE_Y:
                gluLookAt(0.0,  0.0,    0.0,
                          0.0,  0.0,    1.0,
                          0.0,  -1.0,    0.0);
                break;

            case GL_TEXTURE_CUBE_MAP_POSITIVE_Z:
                gluLookAt(0.0,  0.0,    0.0,
                          0.0,  1.0,    0.0,
                          0.0,  0.0,    1.0);
                break;

            case GL_TEXTURE_CUBE_MAP_NEGATIVE_Z:
                gluLookAt(0.0,  0.0,    0.0,
                          0.0,  -1.0,   0.0,
                          0.0,  0.0,    1.0);
                break;

            default:
                break;
        };

        // Save matrix
        glGetFloatv(GL_MODELVIEW_MATRIX, m_CubeFaceLookAtMatrices[f]);
    }
}
}   // namespace AntWorld
}   // namespace BoBRobotics