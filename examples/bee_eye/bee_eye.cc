#include "bee_eye_data.h"

// BoB robotics includes
#include "imgproc/bee_eye.h"
#include "imgproc/opencv_unwrap_360.h"
#include "os/keycodes.h"
#include "video/opencvinput.h"
#include "video/panoramic.h"

// Third-party includes
#include "third_party/CLI11.hpp"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

// Make the code a bit more readable
#define makeUnwrapper std::make_unique<ImgProc::OpenCVUnwrap360>

using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;
using namespace std::literals;

int
main(int argc, char **argv)
{
    bool resizeImages = false;
    bool isPanoramic = false;
    std::string cameraName;
    std::unique_ptr<Video::Input> video;
    std::unique_ptr<ImgProc::OpenCVUnwrap360> unwrapper;

    CLI::App app{ "Applies a bee-eye filter to specified video input." };
    app.add_flag(
            "-p,--panoramic",
            [&isPanoramic](size_t count) {
                isPanoramic = (count > 0);
            },
            "Unwrap panoramic images");
    app.add_option("-c,--camera", cameraName,
                   "Specify type of panoramic camera; implies --panoramic");
    app.allow_extras(); // Extra arg is video input device

    // Parse arguments
    CLI11_PARSE(app, argc, argv);
    if (!cameraName.empty()) {
        isPanoramic = true;
    }

    // Either get camera from argument or use default
    switch (app.remaining_size()) {
    case 0:
        // If panoramic camera requested, try to find one, else use default cam
        video = (isPanoramic) ? Video::getPanoramicCamera()
                              : std::make_unique<Video::OpenCVInput>(0);
        break;
    case 1: {
        // Try to parse as number, otherwise assume it's a string
        const auto args = app.remaining();
        try {
            video = std::make_unique<Video::OpenCVInput>(std::stoi(args[0]));
        } catch (std::invalid_argument &) {
            video = std::make_unique<Video::OpenCVInput>(args[0]);
        }
        break;
    }
    default:
        std::cerr << "Error: Invalid arguments.\n\n"
                  << app.help() << "\n";
        return EXIT_FAILURE;
    }

    // Input size is fixed at compile time
    const cv::Size inputSize{ INPUT_WIDTH, INPUT_HEIGHT };

    if (isPanoramic) {
        if (cameraName.empty()) {
            // Get unwrapper for camera
            unwrapper = makeUnwrapper(video->createUnwrapper(inputSize));
        } else {
            // Otherwise get unwrapper by name (e.g. if using a video file)
            unwrapper = makeUnwrapper(video->getOutputSize(), inputSize, cameraName);
        }
    } else if (video->getOutputSize() != inputSize) {
        try {
            video->setOutputSize(inputSize);
        } catch (std::runtime_error &) {
            resizeImages = true;
            std::cerr << "Warning: Could not set video input's resolution; will resize images instead.\n";
        }
    }

    BeeEye::Map eyeMap{ BeeEye::EyeData, BeeEye::EyeSize, BeeEye::ImageSize };
    cv::Mat outputImage, inputImageRaw, inputImageProcessed;
    while (true) {
        bool newFrame;
        try {
            // Check for new frame
            newFrame = video->readFrame(inputImageRaw);
        } catch (std::runtime_error &) {
            std::cerr << "No more frames available: EOF reached or device unplugged.\n";
            break;
        }

        if (newFrame) {
            // Unwrap if needed and get eye view
            if (isPanoramic) {
                unwrapper->unwrap(inputImageRaw, inputImageProcessed);
                eyeMap.getEyeView(inputImageProcessed, outputImage);
            } else if (resizeImages) {
                cv::resize(inputImageRaw, inputImageProcessed, inputSize);
                eyeMap.getEyeView(inputImageProcessed, outputImage);
            } else {
                eyeMap.getEyeView(inputImageRaw, outputImage);
            }
        }

        // Render to screen
        cv::imshow("Bee eye display", outputImage);
        if((cv::waitKeyEx(1) & OS::KeyMask) == OS::KeyCodes::Escape) {
            break;
        }
    }
}
