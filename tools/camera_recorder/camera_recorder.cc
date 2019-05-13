// BoB robotics includes
#include "common/logging.h"
#include "hid/joystick.h"
#include "imgproc/opencv_unwrap_360.h"
#include "robots/norbot.h"
#include "vicon/capture_control.h"
#include "vicon/udp.h"
#include "video/see3cam_cu40.h"

// Standard C++ includes
#include <fstream>

using namespace BoBRobotics;
using namespace BoBRobotics::HID;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Robots;
using namespace BoBRobotics::Video;
using namespace std::literals;

int main()
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
    Norbot motor;

    cv::Mat output;
    cv::Mat unwrapped(unwrapSize, CV_8UC1);

#ifdef VICON_CAPTURE
    // Create Vicon UDP interface
    Vicon::UDPClient<> vicon(51001);

    // Create Vicon capture control interface
    Vicon::CaptureControl viconCaptureControl("192.168.1.100", 3003,
                                              "c:\\users\\ad374\\Desktop");

    // Start capture
    if(!viconCaptureControl.startRecording("camera_recorder")) {
        return EXIT_FAILURE;
    }

    // Open file to log capture data and write header
    std::ofstream data("vicon.csv");
    data << "Filename, Frame, X, Y, Z, Rx, Ry, Rz" << std::endl;
#endif  // VICON_CAPTURE

    // Loop through time until joystick button pressed
    motor.addJoystick(joystick, joystickDeadzone);
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
            const auto &position = objectData.getPosition<>();
            const auto &attitude = objectData.getAttitude<>();

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
    if(!viconCaptureControl.stopRecording("camera_recorder")) {
        return EXIT_FAILURE;
    }
#endif  // VICON_CAPTURE

    return EXIT_SUCCESS;
}
