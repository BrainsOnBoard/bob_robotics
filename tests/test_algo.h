#pragma once

#include "navigation/generate_images.h"

// BoB robotics includes
#include "common/path.h"
#include "common/serialise_matrix.h"
#include "imgproc/mask.h"

// Standard C++ includes
#include <string>
#include <utility>

using namespace BoBRobotics;
using Window = std::pair<size_t, size_t>;

template<class Algo>
void testAlgo(const std::string &filename, ImgProc::Mask mask, Window window)
{
    const auto filepath = Path::getProgramDirectory() / "navigation" / filename;
    const auto trueDifferences = readMatrix<float>(filepath);

    Algo algo{ TestImageSize };
    for (const auto &image : TestImages) {
        algo.train(image, mask);
    }

    if (window == Window{}) {
        window = algo.getFullWindow();
    }
    const auto &differences = algo.getImageDifferences(TestImages[0], mask, window);
    compareFloatMatrices(differences, trueDifferences);
}
