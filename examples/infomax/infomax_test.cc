// Standard C++ includes
#include <iostream>
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "navigation/infomax.h"

// Third-party includes
#include "third_party/path.h"

using namespace BoBRobotics;

int main()
{
    const filesystem::path dbPath = "../../tools/ant_world_db_creator/ant1_route1";
    cv::Size size(2, 2);

    Navigation::InfoMax infomax(size);

    std::cout << "Weights before training: " << std::endl
              << infomax.getWeights() << std::endl << std::endl;

    cv::Mat trainData(size, CV_8U);
    for (int y = 0; y < size.height; y++) {
        for (int x = 0; x < size.width; x++) {
            trainData.at<uchar>(y, x) = 1;
        }
    }
    infomax.train(trainData, false);

    std::cout << "Weights after training: " << std::endl
              << infomax.getWeights() << std::endl;

    // infomax.loadSnapshotsFromPath(dbPath, true);

    // // load image
    // cv::Mat image = cv::imread((dbPath / getRouteDatabaseFilename(10)).str(), CV_LOAD_IMAGE_GRAYSCALE);
    // cv::resize(image, image, size);

    // const auto decisionValue = infomax.decision(image);

    // std::cout << "Score: " << decisionValue << std::endl;
}