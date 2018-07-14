#pragma once

//----------------------------------------------------------------------------
// Macros
//----------------------------------------------------------------------------
#define BUFFER_OFFSET(i) ((void*)(i))

//----------------------------------------------------------------------------
// Constants
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
constexpr float pi = 3.141592654f;
constexpr float degreesToRadians = (pi / 180.0f);
constexpr float radiansToDegrees = (180.0f / pi);
}   // namespace AntWorld
}   // namespace BoBRobotics