// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <array>
#include <fstream>
#include <iostream>
#include <string>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "navigation/infomax.h"

// Third-party includes
#include "third_party/path.h"

using namespace Eigen;
using namespace BoBRobotics;

auto
readTestData(const filesystem::path &filepath)
{
    // Open file
    std::ifstream is(filepath.str(), std::ios::binary);

    // Get the size of the data
    std::array<int32_t, 2> size;
    is.read(reinterpret_cast<char *>(&size), 2 * sizeof(int32_t));

    // Create data array and fill it
    Matrix<double, Dynamic, Dynamic> data(size[0], size[1]);
    is.read(reinterpret_cast<char *>(data.data()), sizeof(double) * data.size());

    return data;
}

int
main(int, char **argv)
{
    // Path where test *.bin files live
    const auto dataPath = filesystem::path(argv[0]).parent_path() / "test_data";

    // Load matrices of weights
    const auto initWeights = readTestData(dataPath / "weights_init.bin");
    const auto outputWeights = readTestData(dataPath / "weights_out.bin");

    // Load training image
    const auto image = cv::imread((dataPath / "image.png").str(), CV_LOAD_IMAGE_GRAYSCALE);

    // Make our InfoMax runner object
    Navigation::InfoMax<double> infomax(image.size(), initWeights);

    const auto &weights = infomax.getWeights();
    std::cout << "Weights before training: " << std::endl
              << weights << std::endl << std::endl;

    infomax.train(image, false);

    std::cout << "Weights after training: " << std::endl
              << infomax.getWeights() << std::endl << std::endl;

    std::cout << "Matlab's weights: " << std::endl
              << outputWeights << std::endl;

    return 0;
}