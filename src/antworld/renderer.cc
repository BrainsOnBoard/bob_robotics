// BoB robotics includes
#include "antworld/renderer.h"
#include "antworld/render_target.h"

// Standard C++ includes
#include <stdexcept>

//------------------------------------------------------------------------
// BoBRobotics::AntWorld::RendererBase
//------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
void RendererBase::renderFirstPersonView(meter_t x, meter_t y, meter_t z,
                                     degree_t yaw, degree_t pitch, degree_t roll,
                                     GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight)
{
    // Set viewport to strip at stop of window
    glViewport(viewportX, viewportY,
               viewportWidth, viewportHeight);

    // Configure perspective projection matrix
    // **TODO** re-implement in Eigen
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90.0,
                   (GLdouble)viewportWidth / (GLdouble)viewportHeight,
                   m_NearClip, m_FarClip);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
    gluLookAt(0.0,  0.0,    0.0,
              0.0,  1.0,    0.0,
              0.0,  0.0,    1.0);

    applyFrame(x, y, z, yaw, pitch, roll);

    // Clear colour and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render geometry
    renderFirstPersonGeometry();
}
//----------------------------------------------------------------------------
void RendererBase::renderFirstPersonView(meter_t x, meter_t y, meter_t z,
                                     degree_t yaw, degree_t pitch, degree_t roll,
                                     RenderTarget &renderTarget, bool bind, bool clear)
{
    // If we should do so, bind
    if(bind) {
        renderTarget.bind();
    }

    // If we should do so, clear
    if(clear) {
        renderTarget.clear();
    }

    // Render view into target
    renderFirstPersonView(x, y, z, yaw, pitch, roll,
                          0, 0, renderTarget.getWidth(), renderTarget.getHeight());

    // If we should do so, unbind
    if(bind) {
        renderTarget.unbind();
    }
}
//----------------------------------------------------------------------------
void RendererBase::renderTopDownView(GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight)
{
    // Set viewport to square at bottom of screen
    glViewport(viewportX, viewportY, viewportWidth, viewportHeight);

    // Get world bounds
    const auto &minBound = getWorld().getMinBound();
    const auto &maxBound = getWorld().getMaxBound();

    // Configure top-down orthographic projection matrix
    // **TODO** re-implement in Eigen
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(minBound[0].value(), maxBound[0].value(),
               minBound[1].value(), maxBound[1].value());

    // Build modelview matrix to centre world
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Render geometry
    renderTopDownGeometry();
}
//----------------------------------------------------------------------------
void RendererBase::renderTopDownView(RenderTarget &renderTarget, bool bind, bool clear)
{
    // If we should do so, bind
    if(bind) {
        renderTarget.bind();
    }

    // If we should do so, clear
    if(clear) {
        renderTarget.clear();
    }

    // Render view into target
    renderTopDownView(0, 0, renderTarget.getWidth(), renderTarget.getHeight());

    // If we should do so, unbind
    if(bind) {
        renderTarget.unbind();
    }
}
//----------------------------------------------------------------------------
RendererBase::RendererBase(GLsizei cubemapSize, GLdouble nearClip, GLdouble farClip)
:   m_CubemapSize(cubemapSize), m_NearClip(nearClip), m_FarClip(farClip)
{

}
//----------------------------------------------------------------------------
void RendererBase::renderPanoramicGeometry()
{
    m_World.render();
}
//----------------------------------------------------------------------------
void RendererBase::renderFirstPersonGeometry()
{
    m_World.render();
}
//----------------------------------------------------------------------------
void RendererBase::renderTopDownGeometry()
{
    m_World.render();
}
//----------------------------------------------------------------------------
void RendererBase::generateCubeFaceLookAtMatrices(GLdouble eyeX, GLdouble eyeY, GLdouble eyeZ,
                                                  GLfloat (&cubeFaceLookAtMatrices)[6][16])
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
                gluLookAt(eyeX,         eyeY,   eyeZ,
                          eyeX + 1.0,   eyeY,   eyeZ,
                          0.0,          0.0,    1.0);
                break;

            case GL_TEXTURE_CUBE_MAP_NEGATIVE_X:
                gluLookAt(eyeX,         eyeY,   eyeZ,
                          eyeX - 1.0,   0.0,    0.0,
                          0.0,          0.0,    1.0);
                break;

            case GL_TEXTURE_CUBE_MAP_POSITIVE_Y:
                gluLookAt(eyeX, eyeY,   eyeZ,
                          eyeX, eyeY,   eyeZ - 1.0,
                          0.0,  1.0,    0.0);
                break;

            case GL_TEXTURE_CUBE_MAP_NEGATIVE_Y:
                gluLookAt(eyeX, eyeY,   eyeZ,
                          eyeX, eyeY,   eyeZ + 1.0,
                          0.0,  -1.0,    0.0);
                break;

            case GL_TEXTURE_CUBE_MAP_POSITIVE_Z:
                gluLookAt(eyeX, eyeY,       eyeZ,
                          eyeX, eyeY + 1.0, eyeZ,
                          0.0,  0.0,        1.0);
                break;

            case GL_TEXTURE_CUBE_MAP_NEGATIVE_Z:
                gluLookAt(eyeX, eyeY,       eyeZ,
                          eyeX, eyeY - 1.0, eyeZ,
                          0.0,  0.0,        1.0);
                break;

            default:
                break;
        };

        // Save matrix
        glGetFloatv(GL_MODELVIEW_MATRIX, cubeFaceLookAtMatrices[f]);
    }
}
//----------------------------------------------------------------------------
void RendererBase::createCubemapRenderTarget(GLuint &fbo, GLuint &cubemapTexture, GLuint &depthBuffer)
{
    // Create FBO for rendering to cubemap and bind
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // Create cubemap and bind
    glGenTextures(1, &cubemapTexture);
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);

    // Create textures for all faces of cubemap
    // **NOTE** even though we don't need top and bottom faces we still need to create them or rendering fails
    // **TODO** it would be better to use native format
    for(unsigned int t = 0; t < 6; t++) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + t, 0, GL_RGB,
                     m_CubemapSize, m_CubemapSize, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    }
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    // Create depth render buffer
    glGenRenderbuffers(1, &depthBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depthBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, m_CubemapSize, m_CubemapSize);

    // Attach depth buffer to frame buffer
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuffer);

    // Check frame buffer is created correctly
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error("Frame buffer not complete");
    }

    // Unbind cube map and frame buffer
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
//----------------------------------------------------------------------------
void RendererBase::applyFrame(meter_t x, meter_t y, meter_t z,
                          degree_t yaw, degree_t pitch, degree_t roll)
{
    glRotatef((float) roll.value(), 0.f, 1.f, 0.f);
    glRotatef((float) pitch.value(), 1.f, 0., 0.f);
    glRotatef((float) yaw.value(), 0.f, 0.f, 1.f);
    glTranslatef((float) -x.value(), (float) -y.value(), (float) -z.value());
}

//------------------------------------------------------------------------
// BoBRobotics::AntWorld::RendererBase
//-----------------------------------------------------------------------
// hfov = hfov/180/2*pi;
// axis([0 14 -hfov hfov -pi/12 pi/3]);
Renderer::Renderer(GLsizei cubemapSize, GLdouble nearClip, GLdouble farClip,
                   degree_t horizontalFOV, degree_t verticalFOV)
:   RendererBase(cubemapSize, nearClip, farClip), m_RenderMesh(horizontalFOV, verticalFOV, 15_deg, 40, 10)
{
    // Create single cubemap for rendering whole scene
    createCubemapRenderTarget(m_FBO, m_CubemapTexture, m_DepthBuffer);

    // Pre-generate lookat matrices to point at cubemap faces
    generateCubeFaceLookAtMatrices(0.0, 0.0, 0.0,
                                   m_CubeFaceLookAtMatrices);
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
                                   GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight,
                                   GLuint drawFBO)
{
    // Configure viewport to cubemap-sized square
    glViewport(0, 0, getCubemapSize(), getCubemapSize());

    // Bind the cubemap FBO for offscreen rendering
    glBindFramebuffer(GL_FRAMEBUFFER, m_FBO);

    // Configure perspective projection matrix
    // **TODO** re-implement in Eigen
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90.0,
                   1.0,
                   getNearClip(), getFarClip());

    glMatrixMode(GL_MODELVIEW);

    // Save ant transform to matrix
    GLfloat antMatrix[16];
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

        // Render geometry
        renderPanoramicGeometry();
    }

    // Rebind draw framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, drawFBO);

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
void Renderer::renderPanoramicView(meter_t x, meter_t y, meter_t z,
                                   degree_t yaw, degree_t pitch, degree_t roll,
                                   RenderTarget &renderTarget, bool bind, bool clear)
{
    // If we should do so, bind
    if(bind) {
        renderTarget.bind();
    }

    // If we should do so, clear
    if(clear) {
        renderTarget.clear();
    }

    // Render view into target
    renderPanoramicView(x, y, z, yaw, pitch, roll,
                        0, 0, renderTarget.getWidth(), renderTarget.getHeight(),
                        renderTarget.getFBO());

    // If we should do so, unbind
    if(bind) {
        renderTarget.unbind();
    }
}
}   // namespace AntWorld
}   // namespace BoBRobotics
