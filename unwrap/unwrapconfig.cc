#include <memory>
#include <opencv2/opencv.hpp>

#include "../common/opencv_unwrap_360.h"
#include "../videoin/opencvinput.h"

const int CROSS_SIZE = 20; // size of calibration cross

// keyboard key codes
const int KB_LEFT = 81;
const int KB_UP = 82;
const int KB_RIGHT = 83;
const int KB_DOWN = 84;
const int KB_ESC = 27;

// number of pixels to move/resize things by in calibration mode
const int BIG_PX_JUMP = 5;

/* draws a line for the calibration cross */
inline void
drawCalibrationLine(cv::Mat &imorig, cv::Point p1, cv::Point p2)
{
    cv::line(imorig, p1, p2, cv::Scalar(0x00, 0xff, 0x00), 2);
}

int
main(int argc, char **argv)
{
    // TODO: add option to calibrate see3cam too
    VideoIn::VideoInput *cam;
    if (argc == 1) {
        // if no args supplied, use default webcam
        cam = new VideoIn::OpenCVInput;
    } else {
        try {
            // if the arg is an int, the user is specifying a camera...
            int dev = std::stoi(argv[1]);
            cam = new VideoIn::OpenCVInput(dev);
        } catch (std::invalid_argument &e) {
            // ...else it's a filename/URL
            cam = new VideoIn::OpenCVInput(argv[1]);
        }
    }

    // create unwrapper
    // TODO: option to load appropriate params from different file
    std::unique_ptr<OpenCVUnwrap360> unwrapper(OpenCVUnwrap360::loadFromFile(
            "defaultparams/webcam.yaml", cv::Size(1280, 720)));

    int pixelJump = BIG_PX_JUMP; // number of pixels to move by for
                                 // calibration (either 1 or 5)

    cv::Mat imorig;
    cv::Mat unwrap(unwrapper->m_UnwrappedResolution, CV_8UC3);
    bool dirtyFlag = false;

    // display remapped camera output on loop until user presses escape
    for (bool runLoop = true; runLoop;) {
        if (!cam->readFrame(imorig)) {
            std::cerr << "Error: Could not read from webcam" << std::endl;
            return 1;
        }

        unwrapper->unwrap(imorig, unwrap);

        // show unwrapped image
        imshow("unwrapped", unwrap);

        // draw calibration cross at what we've chose as the center
        drawCalibrationLine(imorig,
                            cv::Point(unwrapper->m_CentrePixel.x - CROSS_SIZE,
                                      unwrapper->m_CentrePixel.y),
                            cv::Point(unwrapper->m_CentrePixel.x + CROSS_SIZE,
                                      unwrapper->m_CentrePixel.y));
        drawCalibrationLine(imorig,
                            cv::Point(unwrapper->m_CentrePixel.x,
                                      unwrapper->m_CentrePixel.y - CROSS_SIZE),
                            cv::Point(unwrapper->m_CentrePixel.x,
                                      unwrapper->m_CentrePixel.y + CROSS_SIZE));

        // draw inner and outer circles, showing the area which we will
        // unwrap
        circle(imorig,
               unwrapper->m_CentrePixel,
               unwrapper->m_InnerPixel,
               cv::Scalar(0x00, 0x00, 0xff),
               2);
        circle(imorig,
               unwrapper->m_CentrePixel,
               unwrapper->m_OuterPixel,
               cv::Scalar(0xff, 0x00, 0x00),
               2);

        // show the image
        imshow("calibration", imorig);

        // read keypress in
        int key = cv::waitKey(1) & 0xff;
        /*if (key != 0xff)
            cout << "key: " << key << endl;*/
        switch (key) {
        case KB_ESC: // quit program
            runLoop = false;
            break;
        default:
            switch (key) {
            case ' ': // toggle 1px/5px jumps when moving/resizing
                if (pixelJump == BIG_PX_JUMP)
                    pixelJump = 1;
                else
                    pixelJump = BIG_PX_JUMP;
                break;
            case 'w': // make inner circle bigger
                unwrapper->m_InnerPixel += pixelJump;
                unwrapper->create();
                dirtyFlag = true;
                break;
            case 's': // make inner circle smaller
                if (unwrapper->m_InnerPixel > 0) {
                    unwrapper->m_InnerPixel -= pixelJump;
                    unwrapper->m_InnerPixel =
                            std::max(0, unwrapper->m_InnerPixel);
                    unwrapper->create();
                    dirtyFlag = true;
                }
                break;
            case 'q': // make outer circle bigger
                unwrapper->m_OuterPixel += pixelJump;
                unwrapper->create();
                dirtyFlag = true;
                break;
            case 'a': // make outer circle smaller
                if (unwrapper->m_OuterPixel > 0) {
                    unwrapper->m_OuterPixel -= pixelJump;
                    unwrapper->m_OuterPixel =
                            std::max(0, unwrapper->m_OuterPixel);
                    unwrapper->create();
                    dirtyFlag = true;
                }
                break;
            case KB_UP: // move centre up
                unwrapper->m_CentrePixel.y -= pixelJump;
                unwrapper->create();
                dirtyFlag = true;
                break;
            case KB_DOWN: // move centre down
                unwrapper->m_CentrePixel.y += pixelJump;
                unwrapper->create();
                dirtyFlag = true;
                break;
            case KB_LEFT: // move centre left
                unwrapper->m_CentrePixel.x -= pixelJump;
                unwrapper->create();
                dirtyFlag = true;
                break;
            case KB_RIGHT: // move centre right
                unwrapper->m_CentrePixel.x += pixelJump;
                unwrapper->create();
                dirtyFlag = true;
                break;
            }
        }
    }

    if (dirtyFlag) {
        // write params to file
        // in particular we want to remember our calibration settings so we
        // don't have to recalibrate the next time we start the program
        unwrapper->writeFile();
    }

    return 0;
}
