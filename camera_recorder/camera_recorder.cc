#include "../common/joystick.h"
#include "../common/motor_i2c.h"
#include "../common/opencv_unwrap_360.h"
#include "../common/see3cam_cu40.h"

int main()
{
    constexpr float joystickDeadzone = 0.25f;
    
    const std::string device = "/dev/video0";
    See3CAM_CU40 cam(device, See3CAM_CU40::Resolution::_1280x720);

    cam.setExposure(300);
    cam.setBrightness(30);
    
    // Create joystick interface
    Joystick joystick;
    

// Create window
    const unsigned int rawWidth = cam.getWidth() / 2;
    const unsigned int rawHeight = cam.getHeight() / 2;
    const unsigned int unwrapWidth = 450;
    const unsigned int unwrapHeight = 50;

    // Create unwrapper to unwrap camera output
    auto unwrapper = cam.createUnwrapper(cv::Size(rawWidth, rawHeight),
                                         cv::Size(unwrapWidth, unwrapHeight));
    // Create motor interface
    MotorI2C motor;

 cv::Mat output(rawHeight, rawWidth, CV_8UC1);
    cv::Mat unwrapped(unwrapHeight, unwrapWidth, CV_8UC1);

    for(unsigned int x = 0;;x++) {
        // Read joystick
        joystick.read();
        
        // Use joystick to drive motor
        joystick.drive(motor, joystickDeadzone);
        
	if( cam.captureSuperPixelGreyscale(output))
        {
           unwrapper.unwrap(output, unwrapped);

           if((x %  10) == 0) {
               char filename[255];
               sprintf(filename, "image_%u.png", x);

               cv::imwrite(filename,  unwrapped);
           }
        }
    } while(!joystick.isButtonDown(1));
    
    return 0;
}
