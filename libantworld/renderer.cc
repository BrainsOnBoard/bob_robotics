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
Renderer::Renderer(unsigned int cubemapSize, double nearClip, double farClip,
                   degree_t horizontalFOV, degree_t verticalFOV)
:   m_RenderMesh(horizontalFOV, verticalFOV, 15_deg, 40, 10),
    m_CubemapTexture(0), m_FBO(0), m_DepthBuffer(0),
    m_CubemapSize(cubemapSize), m_NearClip(nearClip), m_FarClip(farClip)
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
                     m_CubemapSize, m_CubemapSize, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);
    }
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    // Create depth render buffer
    glGenRenderbuffers(1, &m_DepthBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, m_DepthBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, m_CubemapSize, m_CubemapSize);

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
Renderer::~Renderer()
{
    glDeleteRenderbuffers(1, &m_DepthBuffer);
    glDeleteTextures(1, &m_CubemapTexture);
    glDeleteFramebuffers(1, &m_FBO);
}
//----------------------------------------------------------------------------
void Renderer::renderPanoramicView(meter_t x, meter_t y, meter_t z,
                                   degree_t yaw, degree_t pitch, degree_t roll,
                                   GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight)
{
    // Configure viewport to cubemap-sized square
    glViewport(0, 0, m_CubemapSize, m_CubemapSize);

    // Bind the cubemap FBO for offscreen rendering
    glBindFramebuffer(GL_FRAMEBUFFER, m_FBO);

    // Configure perspective projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90.0,
                   1.0,
                   m_NearClip, m_FarClip);

    glMatrixMode(GL_MODELVIEW);

    // Save ant transform to matrix
    float antMatrix[16];
    glLoadIdentity();
    applyFrame(x, y, z, yaw, pitch, roll);
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
    glViewport(viewportX, viewportY,
               viewportWidth, viewportHeight);

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
void Renderer::renderFirstPersonView(meter_t x, meter_t y, meter_t z,
                                     degree_t yaw, degree_t pitch, degree_t roll,
                                     GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight)
{
    // Set viewport to strip at stop of window
    glViewport(viewportX, viewportY,
               viewportWidth, viewportHeight);

    // Configure perspective projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90.0,
                   (GLfloat)viewportWidth / (GLfloat)viewportHeight,
                   m_NearClip, m_FarClip);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
    gluLookAt(0.0,  0.0,    0.0,
              0.0,  1.0,    0.0,
              0.0,  0.0,    1.0);

    applyFrame(x, y, z, yaw, pitch, roll);

    // Clear colour and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw world
    m_World.render();
}
//----------------------------------------------------------------------------
void Renderer::renderTopDownView(GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight)
{
    // Set viewport to square at bottom of screen
    glViewport(viewportX, viewportY, viewportWidth, viewportHeight);

    // Get world bounds
    const auto &minBound = getWorld().getMinBound();
    const auto &maxBound = getWorld().getMaxBound();

    // Configure top-down orthographic projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(minBound[0].value(), maxBound[0].value(),
               minBound[1].value(), maxBound[1].value());

    // Build modelview matrix to centre world
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Render world
    m_World.render();
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
//----------------------------------------------------------------------------
void Renderer::applyFrame(meter_t x, meter_t y, meter_t z,
                          degree_t yaw, degree_t pitch, degree_t roll)
{
    glRotatef(roll.value(), 0.0, 1.0f, 0.0);
    glRotatef(pitch.value(), 1.0f, 0.0, 0.0);
    glRotatef(yaw.value(), 0.0, 0.0, 1.0f);
    glTranslatef(-x.value(), -y.value(), -z.value());
}
}   // namespace AntWorld
}   // namespace BoBRobotics