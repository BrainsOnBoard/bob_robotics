#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Common includes
#include "../common/opencv_unwrap_360.h"
#include "../common/timer.h"
#include "../common/v4l_camera.h"

void fillRGGB(cv::Mat &bayer)
{
    for (int row = 0; row < bayer.rows; row+=2)
    {
        for (int col = 0; col < bayer.cols; col+=2)
        {
            bayer.at<uint16_t>(row + 1, col) = bayer.at<uint16_t>(row, col + 1);
        }
    }
}

int main()
{
    // Open camera
    const std::string device = "/dev/video" + std::to_string(1);
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

    // Tweak exposure down to improve frame rate
    //cam.setExposure(50);

    // Create window
    const unsigned int rawWidth = cam.getWidth();
    const unsigned int rawHeight = cam.getHeight();
    const unsigned int unwrapWidth = 900;
    const unsigned int unwrapHeight = 100;

    // Create unwrapper to unwrap camera output
    OpenCVUnwrap360 unwrapper(cv::Size(rawWidth, rawHeight), cv::Size(unwrapWidth, unwrapHeight),
                              0.5, 0.434722222, 0.176388889, 0.381944444);

    cv::namedWindow("Raw", CV_WINDOW_NORMAL);
    cv::resizeWindow("Raw", rawWidth, rawHeight);
    cv::namedWindow("Unwrapped", CV_WINDOW_NORMAL);
    cv::resizeWindow("Unwrapped", unwrapWidth, unwrapHeight);

    cv::Mat output(rawHeight, rawWidth, CV_8UC3);
    cv::Mat unwrapped(unwrapHeight, unwrapWidth, CV_8UC3);

    {
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0;; frame++) {
            if(cam.capture(output)) {
                cv::imshow("Raw", output);

                unwrapper.unwrap(output, unwrapped);
                cv::imshow("Unwrapped", unwrapped);
            }
            if(cv::waitKey(1) == 27) {
                break;
            }
        }

        const double msPerFrame = timer.get() / (double)frame;
        std::cout << "FPS:" << 1000.0 / msPerFrame << std::endl;
    }
    return 0;
}

