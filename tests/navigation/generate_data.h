#pragma once

#include "generate_images.h"

// BoB robotics includes
#include "common/path.h"
#include "common/serialise_matrix.h"

// Standard C++ includes
#include <string>
#include <type_traits>

template<class Algo>
void
generateData(const char *filename)
{
    using namespace BoBRobotics;

    Algo algo{ TestImageSize };
    for (const auto &image : TestImages) {
        algo.train(image);
    }

    const auto filepath = Path::getProgramDirectory() / filename;
    const auto &differences = algo.getImageDifferences(TestImages[0]);
    static_assert(std::is_same<const float &, const decltype(differences[0]) &>::value,
                  "Must return floats");
    writeMatrix(filepath, differences);
}
