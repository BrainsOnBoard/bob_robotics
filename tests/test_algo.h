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
    algo.setMask(std::move(mask));
    for (const auto &image : TestImages) {
        algo.train(image);
    }

    if (window == Window{}) {
        window = algo.getFullWindow();
    }
    const auto &differences = algo.getImageDifferences(window, TestImages[0]);
    compareFloatMatrices(differences, trueDifferences);
}
