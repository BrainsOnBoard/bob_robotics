#pragma once

// Standard C++ includes
#include <map>
#include <memory>
#include <string>
#include <vector>

// OpenGL includes
#include <GL/glew.h>
#include <GL/glu.h>

// BoB robotics includes
#include "common/pose.h"

// Libantworld include
#include "surface.h"
#include "texture.h"

// Forward declarations
namespace cv
{
    class Mat;
}

namespace filesystem
{
    class path;
}

namespace BoBRobotics
{
using namespace units::literals;

namespace AntWorld
{
//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::World
//----------------------------------------------------------------------------
//! Provides a means for loading a world stored on disk into OpenGL
class World
{
    using meter_t = units::length::meter_t;

public:
    World() : m_MinBound{0_m, 0_m, 0_m}, m_MaxBound{0_m, 0_m, 0_m}
    {}

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void render() const;
    void load(const filesystem::path &filename, const GLfloat (&worldColour)[3],
              const GLfloat (&groundColour)[3], bool clear = true);
    void loadObj(const filesystem::path &objFilename, float scale = 1.0f,
                 int maxTextureSize = -1, GLint textureFormat = GL_RGB, 
                 bool clear = true);

    const Vector3<meter_t> &getMinBound()
    {
        return m_MinBound;
    }

    const Vector3<meter_t> &getMaxBound()
    {
        return m_MaxBound;
    }

    void setMinBound(const Vector3<meter_t> &minBound){ m_MinBound = minBound; }
    void setMaxBound(const Vector3<meter_t> &maxBound){ m_MaxBound = maxBound; }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    void loadMaterials(const filesystem::path &basePath, const std::string &filename,
                       GLint textureFormat, int maxTextureSize,
                       std::map<std::string, std::tuple<Texture*, Surface::Colour>> &materialNames);

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Array of surfaces making up the model
    std::vector<Surface> m_Surfaces;

    /// Array of textures making up the model
    std::vector<std::unique_ptr<Texture>> m_Textures;

    // World bounds
    Vector3<meter_t> m_MinBound;
    Vector3<meter_t> m_MaxBound;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
