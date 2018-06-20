#pragma once

// Standard C++ includes
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

//----------------------------------------------------------------------------
// World
//----------------------------------------------------------------------------
class World
{
public:
    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool load(const std::string &filename, const GLfloat (&worldColour)[3],
              const GLfloat (&groundColour)[3]);
    bool loadObj(const std::string &objFilename);

    void render() const;

private:
    //------------------------------------------------------------------------
    // Surface
    //------------------------------------------------------------------------
    // Encapsulates a single 'surface' - geometry to be rendered in one draw call using one material
    class Surface
    {
    public:
        Surface();
        ~Surface();

        void bind() const;
        void render() const;

        void uploadPositions(const std::vector<GLfloat> &positions);
        void uploadColours(const std::vector<GLfloat> &positions);
        void uploadTexCoords(const std::vector<GLfloat> &texCoords);

        void uploadTexture(const cv::Mat &texture);

    private:
        //------------------------------------------------------------------------
        // Members
        //------------------------------------------------------------------------
        GLuint m_VAO;
        GLuint m_PositionVBO;
        GLuint m_ColourVBO;
        GLuint m_TexCoordVBO;
        GLuint m_Texture;
        unsigned int m_NumVertices;
    };

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Array of surfaces making up the model
    std::vector<Surface> m_Surfaces;
};