#include "antworld/texture.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorlds::Texture
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
Texture::Texture()
{
    glGenTextures(1, &m_Texture);
}
//----------------------------------------------------------------------------
Texture::~Texture()
{
    glDeleteTextures(1, &m_Texture);
}
//----------------------------------------------------------------------------
void Texture::bind() const
{
    // Bind texture
    glBindTexture(GL_TEXTURE_2D, m_Texture);
}
//----------------------------------------------------------------------------
void Texture::unbind() const
{
    // Unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);
}
//----------------------------------------------------------------------------
void Texture::upload(const cv::Mat &texture, GLint textureFormat)
{
    // Bind texture
    glBindTexture(GL_TEXTURE_2D, m_Texture);

    // Configure texture filtering
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // Upload texture data and generate mipmaps
    glTexImage2D(GL_TEXTURE_2D, 0, textureFormat, texture.cols, texture.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, texture.data);
    glGenerateMipmap(GL_TEXTURE_2D);
}
}   // namespace AntWorld
}   // namespace BoBRobotics
