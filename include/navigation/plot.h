#pragma once

// Standard C++ includes
#include <vector>

namespace BoBRobotics {
namespace Navigation {

//! Plot a line graph of a single RIDF
void
plotRIDF(const std::vector<float> &differences);

} // Navigation
} // BoBRobotics