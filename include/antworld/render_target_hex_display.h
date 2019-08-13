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

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void render(GLint viewportX, GLint viewportY,
                GLsizei viewportWidth, GLsizei viewportHeight) const;

private:
    Surface m_Surface;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
