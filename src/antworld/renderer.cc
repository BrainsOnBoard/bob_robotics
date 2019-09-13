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
RendererBase::RendererBase(GLdouble nearClip, GLdouble farClip)
:   m_NearClip(nearClip), m_FarClip(farClip)
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
void RendererBase::applyFrame(meter_t x, meter_t y, meter_t z,
                          degree_t yaw, degree_t pitch, degree_t roll)
{
    glRotatef((float) roll.value(), 0.f, 1.f, 0.f);
    glRotatef((float) pitch.value(), 1.f, 0., 0.f);
    glRotatef((float) yaw.value(), 0.f, 0.f, 1.f);
    glTranslatef((float) -x.value(), (float) -y.value(), (float) -z.value());
}

//------------------------------------------------------------------------
// BoBRobotics::AntWorld::Renderer
//-----------------------------------------------------------------------
Renderer::Renderer(std::unique_ptr<RenderMesh> renderMesh,
                   GLsizei cubemapSize, GLdouble nearClip, GLdouble farClip)
:   RendererBase(nearClip, farClip), m_RenderMesh(std::move(renderMesh)),
    m_RenderTargetCubemap(cubemapSize)
{
    // Pre-generate lookat matrices to point at cubemap faces
    generateCubeFaceLookAtMatrices(0.0, 0.0, 0.0,
                                    m_CubeFaceLookAtMatrices);
}
//-----------------------------------------------------------------------
Renderer::Renderer(GLsizei cubemapSize, double nearClip, double farClip,
                   degree_t horizontalFOV, degree_t verticalFOV)
:   Renderer(std::make_unique<RenderMeshSpherical>(false, horizontalFOV, verticalFOV, 0_deg, 15_deg + (verticalFOV / 2.0)),
             cubemapSize, nearClip, farClip)
{
}
//-----------------------------------------------------------------------
void Renderer::renderPanoramicView(meter_t x, meter_t y, meter_t z,
                                   degree_t yaw, degree_t pitch, degree_t roll,
                                   GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight,
                                   GLuint drawFBO)
{
    // Configure viewport to cubemap-sized square
    glViewport(0, 0, m_RenderTargetCubemap.getWidth(), m_RenderTargetCubemap.getHeight());

    // Bind the cubemap FBO for offscreen rendering
    m_RenderTargetCubemap.bind();

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
        // Attach correct cube map face
        m_RenderTargetCubemap.attachCubemapFace(f + GL_TEXTURE_CUBE_MAP_POSITIVE_X);

        // Load look at matrix for this cube face
        glLoadMatrixf(m_CubeFaceLookAtMatrices[f]);

        // Multiply this by ant transform
        glMultMatrixf(antMatrix);

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render geometry
        renderPanoramicGeometry();
    }

    // Unbind cubemap render target and rebind drawFBO
    m_RenderTargetCubemap.unbind(drawFBO);

    // Set viewport to strip at stop of window
    glViewport(viewportX, viewportY,
               viewportWidth, viewportHeight);

    // Bind cubemap texture
    glEnable(GL_TEXTURE_CUBE_MAP);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_RenderTargetCubemap.getCubemapTexture());

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0,
               0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Render render mesh
    m_RenderMesh->render();

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
