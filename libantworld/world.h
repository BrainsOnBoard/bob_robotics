#pragma once

// Standard C++ includes
#include <map>
#include <memory>
#include <string>
#include <vector>

// OpenGL includes
#include <GL/glew.h>
#include <GL/glu.h>

// Forward declarations
namespace cv
{
    class Mat;
}

namespace filesystem
{
    class path;
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::World
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
class World
{
public:
    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool load(const std::string &filename, const GLfloat (&worldColour)[3],
              const GLfloat (&groundColour)[3]);
    bool loadObj(const std::string &objFilename, float scale = 1.0f,
                 int maxTextureSize = -1, GLint textureFormat = GL_RGB);

    void render() const;

private:
    //------------------------------------------------------------------------
    // Texture
    //------------------------------------------------------------------------
    class Texture
    {
    public:
        Texture();
        ~Texture();

        //------------------------------------------------------------------------
        // Public API
        //------------------------------------------------------------------------
        void bind() const;
        void upload(const cv::Mat &texture, GLint textureFormat);

    private:
        //------------------------------------------------------------------------
        // Members
        //------------------------------------------------------------------------
        GLuint m_Texture;
    };

    //------------------------------------------------------------------------
    // Surface
    //------------------------------------------------------------------------
    // Encapsulates a single 'surface' - geometry to be rendered in one draw call using one material
    class Surface
    {
    public:
        Surface();
        ~Surface();

        //------------------------------------------------------------------------
        // Public API
        //------------------------------------------------------------------------
        void bind() const;
        void render() const;

        void uploadPositions(const std::vector<GLfloat> &positions);
        void uploadColours(const std::vector<GLfloat> &positions);
        void uploadTexCoords(const std::vector<GLfloat> &texCoords);

        void setTexture(const Texture *texture){ m_Texture = texture; }

    private:
        //------------------------------------------------------------------------
        // Members
        //------------------------------------------------------------------------
        GLuint m_VAO;
        GLuint m_PositionVBO;
        GLuint m_ColourVBO;
        GLuint m_TexCoordVBO;
        unsigned int m_NumVertices;

        const Texture *m_Texture;
    };

    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    bool loadMaterials(const filesystem::path &basePath, const std::string &filename,
                       GLint textureFormat, int maxTextureSize,
                       std::map<std::string, Texture*> &textureNames);

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Array of surfaces making up the model
    std::vector<Surface> m_Surfaces;

    /// Array of textures making up the model
    std::vector<std::unique_ptr<Texture>> m_Textures;
};
}   // namespace AntWorld
}   // namespace BoBRobotics