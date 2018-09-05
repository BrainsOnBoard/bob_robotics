// Standard C++ includes
#include <iostream>
#include <tuple>

// OpenCV
#include "opencv2/opencv.hpp"

// BoB robotics includes
#include "common/image_database.h"
#include "navigation/infomax.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;
namespace plt = matplotlibcpp;

int main()
{
    const cv::Size imSize { 25, 90 };
    InfoMax<> infomax(imSize);

    std::cout << "Training InfoMax network..." << std::endl;
    const filesystem::path routePath = "../../tools/ant_world_db_creator/ant1_route1";
    for (unsigned i = 0; i < 810; i += 10) {
        const auto filename = getRouteDatabaseFilename(i);
        infomax.trainFromFile(routePath / filename, true);
    }

    const filesystem::path testPath = routePath / getRouteDatabaseFilename(10);
    cv::Mat testImage = cv::imread(testPath.str(), CV_LOAD_IMAGE_GRAYSCALE);
    cv::resize(testImage, testImage, imSize);
    const auto result = infomax.getHeading(testImage);
    std::cout << "Best heading: " << static_cast<degree_t>(std::get<0>(result)) << std::endl;

    plt::plot(std::get<2>(result));
    plt::show();

    return 0;
}