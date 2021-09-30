// BoB robotics includes
#include "hid/joystick.h"
#include "hid/robot_control.h"
#include "imgproc/opencv_unwrap_360.h"
#include "robots/robot_type.h"
#include "vicon/capture_control.h"
#include "vicon/udp.h"
#include "video/see3cam_cu40.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <fstream>

using namespace BoBRobotics;
using namespace BoBRobotics::HID;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Robots;
using namespace BoBRobotics::Video;
using namespace std::literals;

int bobMain(int, char **)
{
    constexpr unsigned int recordingInterval = 10;
    constexpr float joystickDeadzone = 0.25f;

    const std::string device = "/dev/video0";
    See3CAM_CU40 cam(device, See3CAM_CU40::Resolution::_1280x720);

    // Delete old images
    system("rm -f image_*.png");

    // Calculate camera input dimensions
    const cv::Size unwrapSize(180, 50);

    // Create bubblescope mask and use to calibrate camera exposure
    const cv::Mat bubblescopeMask = See3CAM_CU40::createBubblescopeMask(cam.getSuperPixelSize());
    cam.autoExposure(bubblescopeMask);

    // Create joystick interface
    Joystick joystick;

    // Create unwrapper to unwrap camera output
    auto unwrapper = cam.createUnwrapper(unwrapSize);

    // Create motor interface
    ROBOT_TYPE motor;

    cv::Mat output;
    cv::Mat unwrapped(unwrapSize, CV_8UC1);

#ifdef VICON_CAPTURE
    // Create Vicon UDP interface
    Vicon::UDPClient<> vicon(51001);

    // Create Vicon capture control interface
    Vicon::CaptureControl viconCaptureControl("192.168.1.100", 3003,
                                              "c:\\users\\ad374\\Desktop");

    // Start capture
    viconCaptureControl.startRecording("camera_recorder");

    // Open file to log capture data and write header
    std::ofstream data("vicon.csv");
    data.exceptions(std::ios::badbit | std::ios::failbit);
    data << "Filename, Frame, X, Y, Z, Rx, Ry, Rz" << std::endl;
#endif  // VICON_CAPTURE

    // Loop through time until joystick button pressed
    HID::addJoystick(motor, joystick, joystickDeadzone);
    for (unsigned int x = 0; !joystick.isDown(JButton::B); x++) {
        // Read joystick
        joystick.update();

        // If we successfully captured a frame
        cam.readGreyscaleFrame(output);

        // Unwrap frame
        unwrapper.unwrap(output, unwrapped);

        // If recording interval has elapsed
        if ((x % recordingInterval) == 0) {
            // Write image file
            char filename[255];
            sprintf(filename, "image_%u.png", x);
            cv::imwrite(filename, unwrapped);

#ifdef VICON_CAPTURE
            // Get tracking data
            auto objectData = vicon.getObjectData();
            const auto &position = objectData.getPose().position();
            const auto &attitude = objectData.getPose().attitude();

            // Write to CSV
            data << filename << ", " << objectData.getFrameNumber() << ", "
                 << position[0].value() << ", " << position[1].value() << ", "
                 << position[2].value() << ", " << attitude[0].value() << ", "
                 << attitude[1].value() << ", " << attitude[2].value() << std::endl;
#endif // VICON_CAPTURE
        }
    }

#ifdef VICON_CAPTURE
    // Stop capture
    viconCaptureControl.stopRecording("camera_recorder");
#endif  // VICON_CAPTURE

    return EXIT_SUCCESS;
}
