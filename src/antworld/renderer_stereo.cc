// BoB robotics includes
#include "antworld/renderer_stereo.h"

// Third-party includes
#include "plog/Log.h"

//------------------------------------------------------------------------
// BoBRobotics::AntWorld::RendererStereo
//------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
RendererStereo::RendererStereo(std::unique_ptr<RenderMesh> renderMeshLeft, std::unique_ptr<RenderMesh> renderMeshRight,
                               GLsizei cubemapSize, GLdouble nearClip, GLdouble farClip, GLdouble eyeSeperation)
:   RendererBase(nearClip, farClip), m_RenderMeshLeft(std::move(renderMeshLeft)),
    m_RenderMeshRight(std::move(renderMeshRight)), m_RenderTargetCubemapLeft(cubemapSize),
    m_RenderTargetCubemapRight(cubemapSize)
{
    // Pre-generate lookat matrices to point at cubemap faces
    generateCubeFaceLookAtMatrices(eyeSeperation / -2.0, 0.0, 0.0,
                                    m_CubeFaceLookAtMatricesLeft);
    generateCubeFaceLookAtMatrices(eyeSeperation / 2.0, 0.0, 0.0,
                                    m_CubeFaceLookAtMatricesRight);
}
//------------------------------------------------------------------------
void RendererStereo::renderPanoramicView(meter_t x, meter_t y, meter_t z,
                                         degree_t yaw, degree_t pitch, degree_t roll,
                                         GLint viewportX, GLint viewportY, GLsizei viewportWidth, GLsizei viewportHeight,
                                         GLuint drawFBO)
{
    // Configure perspective projection matrix
    // **TODO** re-implement in Eigen
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90.0,
                   1.0,
                   getNearClip(), getFarClip());

    glMatrixMode(GL_MODELVIEW);

    // Save ant transform to matrix
    GLfloat agentMatrix[16];
    glLoadIdentity();
    applyFrame(x, y, z, yaw, pitch, roll);
    glGetFloatv(GL_MODELVIEW_MATRIX, agentMatrix);

    // Configure viewport to cubemap-sized square
    glViewport(0, 0, m_RenderTargetCubemapLeft.getWidth(), m_RenderTargetCubemapLeft.getHeight());

    // Render both cubemaps
    renderCubemap(agentMatrix, m_CubeFaceLookAtMatricesLeft, m_RenderTargetCubemapLeft);
    renderCubemap(agentMatrix, m_CubeFaceLookAtMatricesRight, m_RenderTargetCubemapRight);

    // Unbind last render target
    m_RenderTargetCubemapRight.unbind(drawFBO);

    // Enable cubemap texturing
    glEnable(GL_TEXTURE_CUBE_MAP);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0,
               0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Set viewport to left half and render render mesh
    const GLsizei halfViewportWidth = viewportWidth / 2;
    glViewport(viewportX, viewportY,
               halfViewportWidth, viewportHeight);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_RenderTargetCubemapLeft.getCubemapTexture());
    m_RenderMeshLeft->render();

    // Set viewport to right half and render render mesh
    glViewport(viewportX + halfViewportWidth, viewportY,
               halfViewportWidth, viewportHeight);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_RenderTargetCubemapRight.getCubemapTexture());
    m_RenderMeshRight->render();

    // Disable texture coordinate array, cube map texture and cube map texturing!
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    glDisable(GL_TEXTURE_CUBE_MAP);
}
//------------------------------------------------------------------------
void RendererStereo::renderPanoramicView(meter_t x, meter_t y, meter_t z,
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
//------------------------------------------------------------------------
void RendererStereo::renderCubemap(const GLfloat (&agentMatrix)[16], const GLfloat (&cubeFaceLookAtMatrices)[6][16],
                                   RenderTargetCubemap &renderTarget)
{
    // Bind render target
    renderTarget.bind();

    // Loop through each heading we need to render
    for(GLenum f = 0; f < 6; f++) {
        // Attach correct cube map face
        renderTarget.attachCubemapFace(f + GL_TEXTURE_CUBE_MAP_POSITIVE_X);

        // Load look at matrix for this cube face
        glLoadMatrixf(cubeFaceLookAtMatrices[f]);

        // Multiply this by ant transform
        glMultMatrixf(agentMatrix);

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render geometry
        renderPanoramicGeometry();
    }
}
}   // namespace AntWorld
}   // namespace BoBRobotics
