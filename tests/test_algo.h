#pragma once

#include "navigation/generate_images.h"

// BoB robotics includes
#include "common/path.h"
#include "common/serialise_matrix.h"

// Standard C++ includes
#include <string>
#include <utility>

template<class Algo, class... Ts>
void testAlgoRaw(const std::string &filename, cv::Mat mask, Ts&&... args)
{
    using namespace BoBRobotics;

    const auto filepath = Path::getProgramDirectory() / "navigation" / filename;
    const auto trueDifferences = readMatrix<float>(filepath);

    Algo algo{ TestImageSize, std::forward<Ts>(args)... };
    algo.setMaskImage(std::move(mask));
    for (const auto &image : TestImages) {
        algo.train(image);
    }

    const auto &differences = algo.getImageDifferences(TestImages[0]);
    BOB_ASSERT(trueDifferences.size() == differences.size());
    for (int snap = 0; snap < differences.rows(); snap++) {
        for (int col = 0; col < differences.cols(); col++) {
            EXPECT_FLOAT_EQ(differences(snap, col), trueDifferences(snap, col));
        }
    }
}

template<class Algo, class... Ts>
void testAlgo(const std::string &filename, Ts&&... args)
{
    testAlgoRaw<Algo>(filename, {}, std::forward<Ts>(args)...);
}

template<class Algo, class... Ts>
void testAlgoMask(const std::string &filename, Ts&&... args)
{
    testAlgoRaw<Algo>(filename, TestMask, std::forward<Ts>(args)...);
}
