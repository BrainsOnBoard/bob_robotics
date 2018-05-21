#include <map>
#include <numeric>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Common includes
#include "../common/pid.h"
#include "../common/timer.h"
#include "../imgproc/opencv_unwrap_360.h"
#include "../video/see3cam_cu40.h"

using namespace GeNN_Robotics::ImgProc;
using namespace GeNN_Robotics::Video;

namespace
{
enum class Mode
{
    Clamp,
    Shift,
    WhiteBalanceU30,
    WhiteBalanceCoolWhite,
    Greyscale,
};

void setMode(Mode newMode, Mode &mode, cv::Mat &output, cv::Mat &unwrapped) {
    // If switching TO greyscale - recreate images as greyscale
    if(newMode == Mode::Greyscale && mode != Mode::Greyscale) {
        output.create(output.rows, output.cols, CV_8UC1);
        unwrapped.create(unwrapped.rows, unwrapped.cols, CV_8UC1);
    }
    // Otherwise, if switching FROM greyscale - recreate images as colour
    else if(newMode != Mode::Greyscale && mode == Mode::Greyscale) {
        output.create(output.rows, output.cols, CV_8UC3);
        unwrapped.create(unwrapped.rows, unwrapped.cols, CV_8UC3);
    }

    // Update mode
    mode = newMode;
}
}

int main(int argc, char *argv[])
{
    // Open camera
    const unsigned int deviceIndex = (argc > 1) ? std::atoi(argv[1]) : 0;
    const std::string device = "/dev/video" + std::to_string(deviceIndex);
    See3CAM_CU40 cam(device, See3CAM_CU40::Resolution::_1280x720);

    // Enumerate controls supported by camera
    cam.enumerateControls(
        [&cam](const v4l2_queryctrl &control)
        {
            std::cout << control.name << " (" << std::hex << control.id << std::dec << ")" << std::endl;
            if(control.type == V4L2_CTRL_TYPE_INTEGER) {
                std::cout << "\tInteger - min=" << control.minimum << ", max=" << control.maximum << ", step=" << control.step << ", default=" << control.default_value << std::endl;

                int32_t currentValue;
                if(cam.getControlValue(control.id, currentValue)){
                    std::cout << "\tCurrent value=" << currentValue << std::endl;
                }
            }
            else {
                std::cout << "\tUnknown type " << control.type << std::endl;
            }
        });

    // Get initial brightness and exposure
    int32_t brightness = cam.getBrightness();
    int32_t exposure = cam.getExposure();
   
    // Create window
    const unsigned int rawWidth = cam.getWidth() / 2;
    const unsigned int rawHeight = cam.getHeight() / 2;
    const unsigned int unwrapWidth = 450;
    const unsigned int unwrapHeight = 50;

    Mode mode = Mode::Greyscale;
    
    // Create unwrapper to unwrap camera output and mask to use for autoexposure calculation
    auto unwrapper = cam.createUnwrapper(cv::Size(rawWidth, rawHeight),
                                         cv::Size(unwrapWidth, unwrapHeight));

    auto autoExposureMask = cam.createBubblescopeMask(cv::Size(rawWidth, rawHeight));

    cv::namedWindow("Raw", CV_WINDOW_NORMAL);
    cv::resizeWindow("Raw", rawWidth, rawHeight);
    cv::namedWindow("Unwrapped", CV_WINDOW_NORMAL);
    cv::resizeWindow("Unwrapped", unwrapWidth, unwrapHeight);

    cv::Mat output(rawHeight, rawWidth, CV_8UC1);
    cv::Mat unwrapped(unwrapHeight, unwrapWidth, CV_8UC1);

    {
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0;; frame++) {
            bool success = false;
            switch(mode) {
                case Mode::Clamp:
                    success = cam.captureSuperPixelClamp(output);
                    break;
                case Mode::Shift:
                    success = cam.captureSuperPixel(output);
                    break;
                case Mode::WhiteBalanceU30:
                    success = cam.captureSuperPixelWBU30(output);
                    break;
                case Mode::WhiteBalanceCoolWhite:
                    success = cam.captureSuperPixelWBCoolWhite(output);
                    break;
                case Mode::Greyscale:
                    success = cam.captureSuperPixelGreyscale(output);
            }

            if(success) {
                cv::imshow("Raw", output);

                unwrapper.unwrap(output, unwrapped);
                cv::imshow("Unwrapped", unwrapped);
            }

            const int key = cv::waitKey(1);
            if(key == '1') {
                setMode(Mode::Clamp, mode, output, unwrapped);
                std::cout << "Clamp mode" << std::endl;
            }
            else if(key == '2') {
                setMode(Mode::Shift, mode, output, unwrapped);
                std::cout << "Scale mode" << std::endl;
            }
            else if(key == '3') {
                setMode(Mode::WhiteBalanceCoolWhite, mode, output, unwrapped);
                std::cout << "White balance (cool white)" << std::endl;
            }
            else if(key == '4') {
                setMode(Mode::WhiteBalanceU30, mode, output, unwrapped);
                std::cout << "White balance (U30)" << std::endl;
            }
            else if(key == '5') {
                setMode(Mode::Greyscale, mode, output, unwrapped);
                std::cout << "Greyscale" << std::endl;
            }
            else if(key == 'c') {
                char filename[255];
                sprintf(filename, "image_%u.png", frame);
                cv::imwrite(filename, unwrapped);
            }
            else if(key == 'a') {
                cam.autoExposure(autoExposureMask);
            }
            else if(key ==  '-') {
                if(brightness > 1) {
                    brightness--;
                    cam.setBrightness(brightness);
                    std::cout << "Brightness:" << brightness << std::endl;
                }
            }
            else if(key == '+') {
                if(brightness < 40) {
                    brightness++;
                    cam.setBrightness(brightness);
                    std::cout << "Brightness:" << brightness << std::endl;
                }
            }
            else if(key == ',') {
                if(exposure > 1) {
                    exposure--;
                    cam.setExposure(exposure);
                    std::cout << "Exposure:" << exposure << std::endl;
                }
            }
            else if(key == '.') {
                if(exposure < 9999) {
                    exposure++;
                    cam.setExposure(exposure);
                    std::cout << "Exposure:" << exposure << std::endl;
                }
            }
            else if(key == 27) {
                break;
            }
        }

        const double msPerFrame = timer.get() / (double)frame;
        std::cout << "FPS:" << 1000.0 / msPerFrame << std::endl;
    }
    return 0;
}
