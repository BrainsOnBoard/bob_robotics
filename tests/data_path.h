#pragma once

// BoB robotics includes
#include "common/path.h"

inline auto getTestsPath()
{
    return BoBRobotics::Path::getRepoPath() / "tests" / "data";
}
