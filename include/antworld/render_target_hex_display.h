#pragma once

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
    void render() const;

private:
    Surface m_Surface;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
