#include "config.h"
#include "image_input.h"
#include "memory.h"

// BoB robotics includes
#include "imgproc/opencv_unwrap_360.h"
#include "navigation/image_database.h"

// Third-party includes
#include "plog/Log.h"

using namespace BoBRobotics;

int bobMain(int argc, char *argv[])
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

    const Navigation::ImageDatabase database{ config.getOutputPath() };

    // Load required parameters from database's metadata (YAML) file
    const auto &metadata = database.getMetadata();
    std::unique_ptr<ImgProc::OpenCVUnwrap360> unwrapper;
    cv::Size unwrapSize;
    auto camera = metadata["camera"];
    cv::Size cameraSize;
    camera["resolution"] >> cameraSize;
    if (!metadata["imageInput"].empty() && !metadata["imageInput"]["unwrapper"].empty()) {
        std::string cameraName;
        camera["name"] >> cameraName;
        unwrapper = std::make_unique<ImgProc::OpenCVUnwrap360>(cameraSize, config.getUnwrapRes(), cameraName);
        metadata["imageInput"]["unwrapper"] >> *unwrapper;
        unwrapSize = unwrapper->getOutputSize();
    } else {
        unwrapSize = cameraSize;
    }

    // Create image input
    std::unique_ptr<ImageInput> imageInput = createImageInput(config, unwrapSize, std::move(unwrapper));

    // Train InfoMax network with training image database and save weights
    InfoMax infomax(config, imageInput->getOutputSize());
    infomax.trainRoute(database, *imageInput);

    return EXIT_SUCCESS;
}
