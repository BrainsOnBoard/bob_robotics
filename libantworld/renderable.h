#pragma once

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::Renderable
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
class Renderable
{
public:
    virtual ~Renderable(){}

    //----------------------------------------------------------------------------
    // Declared virtuals
    //----------------------------------------------------------------------------
    virtual void render() const = 0;
};
}   // namespace AntWorld
}   // namespace BoBRobotics