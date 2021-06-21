#pragma once

// OpenGL includes
#include <GL/glew.h>

// Forward declarations
namespace cv
{
    class Mat;
}

//------------------------------------------------------------------------
// BoBRobotics::AntWorld::Texture
//------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
class Texture
{
public:
    Texture();
    ~Texture();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void bind() const;
    void unbind() const;
    void upload(const cv::Mat &texture, GLint textureFormat);

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    GLuint m_Texture;
};
}   // namespace AntWorld
}   // namespace BoBRobotics