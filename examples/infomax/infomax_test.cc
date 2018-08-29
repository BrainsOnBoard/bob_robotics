// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <array>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "navigation/infomax.h"

// Third-party includes
#include "third_party/path.h"

using namespace std::literals;
using namespace Eigen;
using namespace BoBRobotics;

template<typename T>
auto
readTestData(const filesystem::path &filepath)
{
    // Open file
    std::ifstream is(filepath.str(), std::ios::binary);
    if (!is.good()) {
        throw std::runtime_error("Could not open "s + filepath.str());
    }

    // Get the size of the data
    std::array<int32_t, 2> size;
    is.read(reinterpret_cast<char *>(&size), 2 * sizeof(int32_t));

    // Create data array and fill it
    Matrix<T, Dynamic, Dynamic> data(size[0], size[1]);
    is.read(reinterpret_cast<char *>(data.data()), sizeof(T) * data.size());

    return data;
}

void
runTest(const filesystem::path &dataPath, int num)
{
    const auto snum = std::to_string(num);
    std::cout << "==== Running test " << snum << " ====" << std::endl;

    // Load matrices of weights
    const auto initWeights = readTestData<double>(dataPath / ("weights_init"s + snum + ".bin"s));
    const auto outputWeights = readTestData<double>(dataPath / ("weights_out"s + snum + ".bin"s));

    // Load training image
    auto imageMatrix = readTestData<uint8_t>(dataPath / ("image"s + snum + ".bin"s));
    const cv::Mat image(imageMatrix.rows(), imageMatrix.cols(),
                        CV_8UC1, reinterpret_cast<void *>(imageMatrix.data()));

    // Make our InfoMax runner object
    Navigation::InfoMax<double> infomax(image.size(), initWeights);

    // Do training
    infomax.train(image, false);

    std::cout << "Weights before training: " << std::endl
              << initWeights << std::endl
              << std::endl;

    Map<Matrix<uint8_t, Dynamic, Dynamic>> map(image.data, image.cols, image.rows);
    std::cout << "Image: " << std::endl
              << map.cast<int>() << std::endl
              << std::endl;

    std::cout << "Weights after training: " << std::endl
              << infomax.getWeights() << std::endl
              << std::endl;

    std::cout << "Matlab's weights: " << std::endl
              << outputWeights << std::endl;
    std::cout << "==========" << std::endl;
}

int
main(int, char **argv)
{
    // Path where test *.bin files live
    const auto dataPath = filesystem::path(argv[0]).parent_path() / "test_data";

    // Run tests
    runTest(dataPath, 1);

    return 0;
}