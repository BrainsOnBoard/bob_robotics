#include "read_data.h"

// BoB robotics includes
#include "common/logging.h"
#define EXPOSE_INFOMAX_INTERNALS
#include "navigation/infomax.h"

// Third-party includes
#include "third_party/path.h"

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <array>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

using namespace std::literals;
using namespace Eigen;
using namespace BoBRobotics::Navigation;

void
runTest(const filesystem::path &dataPath, int num)
{
    const int manyRuns = 100;

    const auto snum = std::to_string(num);
    LOGI << "==== Running test " << snum << " ====";

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
    InfoMaxRotater<InSilicoRotater, double> infomax(image.size(), initWeights);

    // Do training
    Matrix<double, Dynamic, 1> u, y;
    infomax.calculateUY(image);
    std::tie(u, y) = infomax.getUY();
    infomax.trainUY();

    LOGI << "Weights before training: "
              << initWeights
             ;

    LOGI << "Image: "
              << imageMatrix.cast<int>()
             ;

    LOGI << "U: " << u;
    LOGI << "Y: " << y;

    const auto &weights = infomax.getWeights();
    LOGI << "Weights after training: "
              << weights
             ;

    LOGI << "Diff U: " << matlabU - u;
    LOGI << "Diff Y: " << matlabY - y;
    LOGI << "Diff weights: " << matlabOutputWeights - weights;

    for (int i = 1; i < manyRuns; i++) {
        infomax.train(image);
    }
    LOGI << "Diff weights (" << manyRuns << " runs): "
              << matlabOutputWeightsMany - infomax.getWeights();
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