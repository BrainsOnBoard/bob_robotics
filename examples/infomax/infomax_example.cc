// Standard C++ includes
#include <iostream>

// OpenCV
#include "opencv2/opencv.hpp"

// BoB robotics includes
#include "common/image_database.h"
#include "navigation/infomax.h"

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;

int main()
{
    const cv::Size imSize { 25, 90 };
    InfoMax<> infomax(imSize);

    std::cout << "Training InfoMax network..." << std::endl;
    const filesystem::path routePath = "../../tools/ant_world_db_creator/ant1_route1";
    for (unsigned i = 0; i < 100; i++) {
        const auto filename = getRouteDatabaseFilename(i);
        infomax.trainFromFile(routePath / filename, true);
    }
}