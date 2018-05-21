// Standard C++ includes
#include <fstream>

// GeNN robotics includes
#include "../common/joystick.h"
#include "../common/motor_i2c.h"
#include "../common/vicon_capture_control.h"
#include "../common/vicon_udp.h"
#include "../imgproc/opencv_unwrap_360.h"
#include "../video/see3cam_cu40.h"

using namespace GeNNRobotics;
using namespace GeNNRobotics::ImgProc;
using namespace GeNNRobotics::Video;

int main()
{
    constexpr unsigned int recordingInterval = 10;
    constexpr float joystickDeadzone = 0.25f;

    const std::string device = "/dev/video0";
    See3CAM_CU40 cam(device, See3CAM_CU40::Resolution::_1280x720);

    // Delete old images
    system("rm -f image_*.png");

    // Calculate camera input dimensions
    const unsigned int rawWidth = cam.getWidth() / 2;
    const unsigned int rawHeight = cam.getHeight() / 2;
    const unsigned int unwrapWidth = 180;
    const unsigned int unwrapHeight = 50;

    // Create bubblescope mask and use to calibrate camera exposure
    const cv::Mat bubblescopeMask = See3CAM_CU40::createBubblescopeMask(cv::Size(rawWidth, rawHeight));
    cam.autoExposure(bubblescopeMask);

    // Create joystick interface
    Joystick joystick;

    // Create unwrapper to unwrap camera output
    auto unwrapper = cam.createUnwrapper(cv::Size(rawWidth, rawHeight),
                                         cv::Size(unwrapWidth, unwrapHeight));
    // Create motor interface
    MotorI2C motor;

    cv::Mat output(rawHeight, rawWidth, CV_8UC1);
    cv::Mat unwrapped(unwrapHeight, unwrapWidth, CV_8UC1);

#ifdef VICON_CAPTURE
    // Create Vicon UDP interface
    Vicon::UDPClient<Vicon::ObjectData> vicon(51001);

    // Create Vicon capture control interface
    Vicon::CaptureControl viconCaptureControl("192.168.1.100", 3003,
                                              "c:\\users\\ad374\\Desktop");

    // Wait for tracking
    while(vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Waiting for object" << std::endl;
    }

    // Start capture
    if(!viconCaptureControl.startRecording("camera_recorder")) {
        return EXIT_FAILURE;
    }

    // Open file to log capture data and write header
    std::ofstream data("vicon.csv");
    data << "Filename, Frame, X, Y, Z, Rx, Ry, Rz" << std::endl;
#endif  // VICON_CAPTURE

    // Loop through time until joystick button pressed
    for(unsigned int x = 0; !joystick.isButtonDown(1); x++) {
        // Read joystick
        joystick.read();
        
        // Use joystick to drive motor
        joystick.drive(motor, joystickDeadzone);

        // If we successfully captured a frame
        if(cam.captureSuperPixelGreyscale(output))
        {
           // Unwrap frame
           unwrapper.unwrap(output, unwrapped);

           // If recording interval has elapsed
           if((x % recordingInterval) == 0) {
               // Write image file
               char filename[255];
               sprintf(filename, "image_%u.png", x);
               cv::imwrite(filename,  unwrapped);

#ifdef VICON_CAPTURE
               // Get tracking data
               auto objectData = vicon.getObjectData(0);
               const auto &translation = objectData.getTranslation();
               const auto &rotation = objectData.getRotation();

               // Write to CSV
               data << filename << ", " << objectData.getFrameNumber() << ", " << translation[0] << ", " << translation[1] << ", " << translation[2] << ", " << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << std::endl;
#endif  // VICON_CAPTURE
           }
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
