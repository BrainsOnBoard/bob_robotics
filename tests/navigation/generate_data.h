#pragma once

#include "generate_images.h"

// BoB robotics includes
#include "common/path.h"
#include "common/serialise_matrix.h"

// Standard C++ includes
#include <string>
#include <type_traits>
#include <utility>

template<class Algo, class... Ts>
void
generateData(const char *filename, Ts&&... args)
{
    using namespace BoBRobotics;

    Algo algo{ TestImageSize, std::forward<Ts>(args)... };
    for (const auto &image : TestImages) {
        algo.train(image);
    }

    const auto filepath = Path::getProgramDirectory() / filename;
    const auto &differences = algo.getImageDifferences(TestImages[0]);
    static_assert(std::is_same<const float &, const decltype(differences[0]) &>::value,
                  "Must return floats");
    writeMatrix(filepath, differences);
}
