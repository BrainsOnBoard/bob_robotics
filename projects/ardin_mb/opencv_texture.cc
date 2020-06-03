#include "opencv_texture.h"


// OpenCV includes
#include <opencv2/opencv.hpp>

//----------------------------------------------------------------------------
// OpenCVTexture
//----------------------------------------------------------------------------
OpenCVTexture::OpenCVTexture(GLint filtering)
{
    // Generate texture handle
    glGenTextures(1, &m_Texture);

    // Bind texture handle
    glBindTexture(GL_TEXTURE_2D, m_Texture);

    // Set filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filtering);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filtering);

    // Unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);

}
//----------------------------------------------------------------------------
OpenCVTexture::~OpenCVTexture()
{
    // Delete texture
    glDeleteTextures(1, &m_Texture);
}
//----------------------------------------------------------------------------
void OpenCVTexture::update(const cv::Mat &image)
{
    // Convert OpenCV type to
    GLenum internalFormat;
    GLenum dataFormat;
    if(image.type() == CV_8UC3) {
        internalFormat = dataFormat = GL_BGR;
    }
    else if(image.type() == CV_8UC1) {
        internalFormat = dataFormat = GL_LUMINANCE;
    }
    else {
        throw std::runtime_error("Unsupport OpenCV data type:" + std::to_string(image.type()));
    }

    // Bind texture handle
    glBindTexture(GL_TEXTURE_2D, m_Texture);

    // Upload image data
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, image.cols, image.rows, 0, dataFormat, GL_UNSIGNED_BYTE, image.data);

    // Unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);

}