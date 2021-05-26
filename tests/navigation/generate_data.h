#pragma once

#include "generate_images.h"

// BoB robotics includes
#include "common/path.h"
#include "common/serialise_matrix.h"
#include "imgproc/mask.h"

// Standard C++ includes
#include <string>
#include <type_traits>
#include <utility>

using namespace BoBRobotics;
using Window = std::pair<size_t, size_t>;

template<class Algo, class... Ts>
void
generateDataRaw(const std::string &filename, ImgProc::Mask mask, Window window,
                Ts&&... args)
{
    Algo algo{ TestImageSize, std::forward<Ts>(args)... };
    algo.setMask(std::move(mask));
    for (const auto &image : TestImages) {
        algo.train(image);
    }

    if (window == Window{}) {
        window = algo.getFullWindow();
    }
    const auto &differences = algo.getImageDifferences(window, TestImages[0]);
    static_assert(std::is_same<const float &, const decltype(differences[0]) &>::value,
                  "Must return floats");
    writeMatrix(Path::getProgramDirectory() / filename, differences);
}

template<class Algo, class... Ts>
void
generateData(const std::string &filename, Ts&&... args)
{
    generateDataRaw<Algo>(filename, {}, {}, std::forward<Ts>(args)...);
    generateDataRaw<Algo>("mask_" + filename, TestMask, {}, std::forward<Ts>(args)...);

    const Window window{ 0, 10 };
    generateDataRaw<Algo>("window_" + filename, {}, window, std::forward<Ts>(args)...);
    generateDataRaw<Algo>("window_mask_" + filename, TestMask, window, std::forward<Ts>(args)...);
}
