// Standard C++ includes
#include <iostream>

// OpenCV
#include "opencv2/opencv.hpp"

// BoB robotics includes
#include "navigation/infomax.h"

using namespace BoBRobotics::Navigation;

int main()
{
    const cv::Size imSize { 25, 90 };
    InfoMax<> infomax(imSize);

    std::cout << "Training InfoMax network..." << std::endl;
    infomax.trainRoute("../../tools/ant_world_db_creator/ant1_route1", true);
}