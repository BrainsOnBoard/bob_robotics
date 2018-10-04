#include "read_data.h"

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <array>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#define EXPOSE_INFOMAX_INTERNALS
#include "navigation/infomax.h"

// Third-party includes
#include "third_party/path.h"

using namespace std::literals;
using namespace Eigen;
using namespace BoBRobotics::Navigation;

void
runTest(const filesystem::path &dataPath, int num)
{
    const int manyRuns = 100;

    const auto snum = std::to_string(num);
    std::cout << "==== Running test " << snum << " ====" << std::endl;

    // Load matrices of weights
    const auto pref = "test"s + snum + "_"s;
    const auto initWeights = readData<>(dataPath / (pref + "weights_init.bin"s));
    const auto matlabOutputWeights = readData<>(dataPath / (pref + "weights_out.bin"s));
    const auto matlabOutputWeightsMany = readData<>(dataPath / (pref + "weights_out_many.bin"));
    const auto matlabU = readData<>(dataPath / (pref + "u.bin"s));
    const auto matlabY = readData<>(dataPath / (pref + "y.bin"s));

    // Load training image
    auto imageMatrix = readData<uint8_t>(dataPath / (pref + "train_image.bin"s));
    const cv::Mat image(imageMatrix.rows(), imageMatrix.cols(),
                        CV_8UC1, reinterpret_cast<void *>(imageMatrix.data()));

    // Make our InfoMax runner object
    InfoMax<InSilicoRotater, double> infomax(image.size(), initWeights);

    // Do training
    Matrix<double, Dynamic, 1> u, y;
    std::tie(u, y) = infomax.getUY(image);
    infomax.trainUY(u, y);

    std::cout << "Weights before training: " << std::endl
              << initWeights << std::endl
              << std::endl;

    std::cout << "Image: " << std::endl
              << imageMatrix.cast<int>() << std::endl
              << std::endl;

    std::cout << "U: " << u << std::endl << std::endl;
    std::cout << "Y: " << y << std::endl << std::endl;

    const auto &weights = infomax.getWeights();
    std::cout << "Weights after training: " << std::endl
              << weights << std::endl
              << std::endl;

    std::cout << "Diff U: " << std::endl << matlabU - u << std::endl << std::endl;
    std::cout << "Diff Y: " << std::endl << matlabY - y << std::endl << std::endl;
    std::cout << "Diff weights: " << matlabOutputWeights - weights << std::endl << std::endl;

    for (int i = 1; i < manyRuns; i++) {
        infomax.train(image);
    }
    std::cout << "Diff weights (" << manyRuns << " runs): "
              << matlabOutputWeightsMany - infomax.getWeights() << std::endl << std::endl;
}

int
main(int, char **argv)
{
    // Path where test *.bin files live
    const auto dataPath = filesystem::path(argv[0]).parent_path() / "test_data";

    // Run tests
    runTest(dataPath, 1);
    runTest(dataPath, 2);
    runTest(dataPath, 3);

    return 0;
}