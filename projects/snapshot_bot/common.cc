#include "common.h"

// BoB robotics includes
#include "video/opencvinput.h"
#include "video/panoramic.h"
#ifdef USE_ODK2
#include "video/odk2/odk2.h"
#endif

// Third-party includes
#include "plog/Log.h"

using namespace BoBRobotics;

std::unique_ptr<Video::Input> getPanoramicCamera(const Config &config)
{
    if (config.shouldUseWebcam()) {
        return std::make_unique<Video::OpenCVInput>();
    } else if (config.shouldUseODK2()) {
#ifdef USE_ODK2
        return std::make_unique<Video::ODK2>();
#else
        throw std::runtime_error("Snapshot bot not compiled with ODK2 support - please re-run cmake with -DUSE_ODK2=1");
#endif
    } else {
        return Video::getPanoramicCamera(cv::CAP_V4L);
    }
}

std::unique_ptr<ImageInput> createImageInput(const Config &config, const Video::Input &camera)
{
    // If camera image will need unwrapping, create unwrapper
    std::unique_ptr<ImgProc::OpenCVUnwrap360> unwrapper;
    cv::Size unwrapSize;
    if (camera.needsUnwrapping()) {
        unwrapper = std::make_unique<ImgProc::OpenCVUnwrap360>(camera.createUnwrapper(config.getUnwrapRes()));
        unwrapSize = unwrapper->getOutputSize();
    } else {
        unwrapSize = camera.getOutputSize();
    }

    return ::createImageInput(config, unwrapSize, std::move(unwrapper));
}
