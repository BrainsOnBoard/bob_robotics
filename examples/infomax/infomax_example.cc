// Standard C++ includes
#include <iostream>
#include <tuple>

// Eigen
#include <Eigen/Core>

// OpenCV
#include "opencv2/opencv.hpp"

// BoB robotics includes
#include "common/timer.h"
#include "navigation/image_database.h"
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
    const ImageDatabase routeImages("../../tools/ant_world_db_creator/ant1_route1");

    std::cout << "Eigen is using " << Eigen::nbThreads() << " threads." << std::endl
              << std::endl;

    // Object to run InfoMax algorithm
    InfoMax<> infomax(imSize);

    // Train network
    {
        std::cout << "Training InfoMax network..." << std::endl;
        Timer<> trainingTimer{ "Network trained in: " };
        for (unsigned i = 0; i < routeImages.size(); i += 50) {
            infomax.VisualNavigationBase::train(routeImages[i].loadGreyscale(), true);
        }
    }

    // Get output
    cv::Mat testImage = routeImages[10].loadGreyscale();
    cv::resize(testImage, testImage, imSize);
    const auto result = infomax.getHeading(testImage);
    std::cout << std::endl << "Best heading: "
              << static_cast<degree_t>(std::get<0>(result)) << std::endl;

    plt::plot(std::get<2>(result));
    plt::xlim(0, 90);
    plt::show();

    return 0;
}