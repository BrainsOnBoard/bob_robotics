// Standard C++ includes
#include <iostream>
#include <tuple>

// Eigen
#include <Eigen/Core>

// OpenCV
#include "opencv2/opencv.hpp"

// BoB robotics includes
#include "common/image_database.h"
#include "common/timer.h"
#include "navigation/infomax.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;
namespace plt = matplotlibcpp;

int
main()
{
    const cv::Size imSize{ 90, 25 };
    const filesystem::path routePath = "../../tools/ant_world_db_creator/ant1_route1";

    std::cout << "Eigen is using " << Eigen::nbThreads() << " threads." << std::endl
              << std::endl;

    // Object to run InfoMax algorithm
    InfoMax<double> infomax(imSize);

    // Train network
    {
        std::cout << "Training InfoMax network..." << std::endl;
        Timer<> trainingTimer{ "Network trained in: " };
        for (unsigned i = 0; i < 810; i += 10) {
            const auto filename = getRouteDatabaseFilename(i);
            infomax.trainFromFile(routePath / filename, true);
        }
    }

    // Get output
    const filesystem::path testPath = routePath / getRouteDatabaseFilename(10);
    cv::Mat testImage = cv::imread(testPath.str(), CV_LOAD_IMAGE_GRAYSCALE);
    cv::resize(testImage, testImage, imSize);
    const auto result = infomax.getHeading(testImage);
    std::cout << std::endl << "Best heading: "
              << static_cast<degree_t>(std::get<0>(result)) << std::endl;

    plt::plot(std::get<2>(result));
    plt::xlim(0, 90);
    plt::show();

    return 0;
}