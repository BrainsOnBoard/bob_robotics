#include "config.h"
#include "image_input.h"
#include "memory.h"

// BoB robotics includes
#include "navigation/image_database.h"

// Third-party includes
#include "plog/Log.h"

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

    // Create image input
    std::unique_ptr<ImageInput> imageInput = createImageInput(config);

    // Train InfoMax network with training image database and save weights
    InfoMax infomax(config, imageInput->getOutputSize());
    BoBRobotics::Navigation::ImageDatabase database{ config.getOutputPath() };
    infomax.trainRoute(database, config.shouldUseODK2(), *imageInput);

    return EXIT_SUCCESS;
}
