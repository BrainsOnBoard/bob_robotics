// BoB robotics includes
#include "navigation/plot.h"

// Standard C++ includes
#include <algorithm>

// Third-party includes
#include "third_party/matplotlibcpp.h"

namespace BoBRobotics {
namespace Navigation {

namespace plt = matplotlibcpp;

//! Plot a line graph of a single RIDF
void
plotRIDF(const std::vector<float> &differences)
{
    // Vector to store modified RIDF
    std::vector<float> ridf;
    ridf.reserve(differences.size() + 1);

    // Rotate so data is in range -180 to 180
    std::rotate_copy(differences.begin(),
                     differences.begin() + ceil((float) differences.size() / 2.0f),
                     differences.end(),
                     std::back_inserter(ridf));

    // Normalise to be between 0 and 1
    for (auto &d : ridf) {
        d /= 255.0f;
    }

    // Copy value from -180 to 180 so RIDF is symmetrical
    ridf.push_back(ridf[0]);

    // Calculate x values (angles)
    std::vector<float> x;
    x.reserve(ridf.size());
    for (size_t i = 0; i < ridf.size(); i++) {
        x.push_back(-180.0f + (float) i * 360.0f / (ridf.size() - 1));
    }

    // Plot RIDF
    plt::plot(x, ridf);
    plt::ylabel("Image difference");
    plt::xlabel("Angle (deg)");
    plt::xlim(-180, 180);
    plt::ylim(0.0, plt::ylim()[1]);
    plt::show();
}
} // Navigation
} // BoBRobotics