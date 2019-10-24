#pragma once

// Standard C++ includes
#include <vector>

namespace BoBRobotics {
namespace NavigationPlotting {

//! Plot a line graph of a single RIDF
void
plotRIDF(const std::vector<float> &differences);

} // NavigationPlotting
} // BoBRobotics