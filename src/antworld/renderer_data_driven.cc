// BoB robotics includes
#include "antworld/renderer_data_driven.h"

// Third-party includes
#include "plog/Log.h"

//------------------------------------------------------------------------
// BoBRobotics::AntWorld::RendererDataDriven
//------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
RendererDataDriven::RendererDataDriven(const std::string &eyeBorderFilename, units::angle::degree_t interommatidiaAngle,
                                       GLsizei cubemapSize, GLdouble nearClip, GLdouble farClip)
:   RendererBase(nearClip, farClip), m_RenderMesh(eyeBorderFilename, interommatidiaAngle),
    m_RenderTargetCubemap(cubemapSize)
{
    // Pre-generate lookat matrices to point at cubemap faces
    generateCubeFaceLookAtMatrices(0.0, 0.0, 0.0,
                                   m_CubeFaceLookAtMatrices);
}
//------------------------------------------------------------------------
void RendererDataDriven::renderPanoramicView(meter_t x, meter_t y, meter_t z,
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
    glViewport(0, 0, m_RenderTargetCubemap.getWidth(), m_RenderTargetCubemap.getHeight());

    // Bind render target
    m_RenderTargetCubemap.bind();

    // Loop through each heading we need to render
    for(GLenum f = 0; f < 6; f++) {
        // Attach correct cube map face
        m_RenderTargetCubemap.attachCubemapFace(f + GL_TEXTURE_CUBE_MAP_POSITIVE_X);

        // Load look at matrix for this cube face
        glLoadMatrixf(m_CubeFaceLookAtMatrices[f]);

        // Multiply this by ant transform
        glMultMatrixf(agentMatrix);

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render geometry
        renderPanoramicGeometry();
    }

    // Unbind last render target
    m_RenderTargetCubemap.unbind(drawFBO);

    // Set viewport
    glViewport(viewportX, viewportY,
               viewportWidth, viewportHeight);

    // Enable cubemap texturing
    glEnable(GL_TEXTURE_CUBE_MAP);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0,
               0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glBindTexture(GL_TEXTURE_CUBE_MAP, m_RenderTargetCubemap.getCubemapTexture());

    // Render render mesh
    m_RenderMesh.render();

    // Disable texture coordinate array, cube map texture and cube map texturing!
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    glDisable(GL_TEXTURE_CUBE_MAP);
}
//------------------------------------------------------------------------
void RendererDataDriven::renderPanoramicView(meter_t x, meter_t y, meter_t z,
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
