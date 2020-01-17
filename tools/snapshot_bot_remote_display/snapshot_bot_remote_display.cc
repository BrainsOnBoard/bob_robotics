// OpenCV
#include <opencv2/opencv.hpp>

// Snapshot bot display includes
#include "config.h"

//using namespace BoBRobotics;

int main(int argc, char *argv[])
{
    const char *configFilename = (argc > 1) ? argv[1] : "config.yaml";

    // Read config values from file
    Config config;
    {
        cv::FileStorage configFile(configFilename, cv::FileStorage::READ);
        if(configFile.isOpened()) {
            configFile["config"] >> config;
        }
    }

    // Re-write config file
    {
        cv::FileStorage configFile(configFilename, cv::FileStorage::WRITE);
        configFile << "config" << config;
    }
}
