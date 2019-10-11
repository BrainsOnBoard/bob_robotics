#pragma once

// Ant world includes
#include "antworld/render_target.h"
#include "antworld/surface.h"

// Forward declarations
namespace BoBRobotics
{
namespace AntWorld
{
class RenderMeshHexagonal;
}
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RenderTarget
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
class RenderTargetHexDisplay : public RenderTarget
{
public:
    RenderTargetHexDisplay(const RenderMeshHexagonal &renderMesh);
    RenderTargetHexDisplay(const RenderMeshHexagonal &renderMeshLeft, const RenderMeshHexagonal &renderMeshRight);

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void render(GLint viewportX, GLint viewportY,
                GLsizei viewportWidth, GLsizei viewportHeight) const;
    void render(RenderTarget &renderTarget, bool bind = true, bool clear = true) const;

private:
    RenderTargetHexDisplay(unsigned int numHorizontalHexes, unsigned int numVerticalHexes);

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    Surface m_Surface;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
