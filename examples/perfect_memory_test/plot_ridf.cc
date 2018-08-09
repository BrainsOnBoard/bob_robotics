// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <iostream>

// BoB robotics includes
#include "navigation/perfect_memory.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace BoBRobotics;

int main()
{
    // Class to run perfect memory algorithm
    cv::Size imSize(180, 50);
    Navigation::PerfectMemory pm(imSize);

    // Load a single snapshot
    cv::Mat snap = cv::imread("../../ant_world_db_creator/ant1_route1/image_00010.png", CV_LOAD_IMAGE_GRAYSCALE);
    cv::resize(snap, snap, imSize);
    pm.train(snap);

    // Compare snapshot with itself
    const auto result = pm.getRIDF(snap);
    auto ridf = std::get<3>(result);

    // Rotate so the range is from -180 to 180 deg
    std::rotate(ridf.begin(), ridf.begin() + ceil((double) ridf.size() / 2.0), ridf.end());

    // Copy value from -180 deg to 180 deg
    ridf.push_back(ridf[0]);

    // Normalise values so they are between 0 and 1
    std::transform(ridf.begin(), ridf.end(), ridf.begin(), [](float f) {
        return f / 255.0f;
    });

    // Calculate x values (angles)
    std::vector<float> x(ridf.size());
    for (size_t i = 0; i < x.size(); i++) {
        x[i] = -180.0f + (float) i *  360.0f / (x.size() - 1);
    }

    // Plot RIDF
    plt::plot(x, ridf);
    plt::ylabel("Image difference");
    plt::xlabel("Angle (deg)");
    plt::xlim(-180, 180);
    plt::show();
}