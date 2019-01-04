#pragma once

// OpenGL includes
#include <GL/glew.h>

// Forward declarations
namespace cv
{
    class Mat;
}

//----------------------------------------------------------------------------
// OpenCVTexture
//----------------------------------------------------------------------------
//! Simple wrapper class for uploading an OpenCV image to an OpenGL texture handle
//! e.g. to allow it to be rendered using ImGui
class OpenCVTexture
{
public:
    OpenCVTexture(GLint filtering = GL_NEAREST);
    ~OpenCVTexture();

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(const cv::Mat &image);
    GLuint getTexture() const
    {
        return m_Texture;
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    GLuint m_Texture;

};