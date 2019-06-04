// Standard C++ includes
#include <atomic>
#include <chrono>
#include <thread>

// Common includes
#include "common/lm9ds1_imu.h"
#include "common/logging.h"
#include "common/timer.h"
#include "hid/joystick.h"
#include "imgproc/opencv_optical_flow.h"
#include "imgproc/opencv_unwrap_360.h"
#include "robots/tank.h"
#include "video/see3cam_cu40.h"

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

// Model includes
#include "parameters.h"
#include "robotCommon.h"
#include "robotParameters.h"

using namespace BoBRobotics;
using namespace BoBRobotics::StoneCX;
using namespace BoBRobotics::HID;
using namespace BoBRobotics::ImgProc;
using namespace BoBRobotics::Robots;
using namespace BoBRobotics::Video;

//---------------------------------------------------------------------------
// Anonymous namespace
//---------------------------------------------------------------------------
namespace
{
void buildOpticalFlowFilter(cv::Mat &filter, float preferredAngle) {
    // Loop through columns
    for(int x = 0; x < filter.cols; x++) {
        // Convert column to angle
        const float th = (((float)x / (float)filter.cols) * 2.0f * Parameters::pi) - Parameters::pi;

        // Write filter with sin of angle
        filter.at<float>(0, x) = sin(th - preferredAngle);
    }
}

void imuThreadFunc(std::atomic<bool> &shouldQuit, std::atomic<float> &heading, unsigned int &numSamples)
{
    // Create IMU interface
    LM9DS1 imu;

    // Initialise IMU magnetometer
    LM9DS1::MagnetoSettings magSettings;
    imu.initMagneto(magSettings);

    // While quit signal isn't set
    for(numSamples = 0; !shouldQuit; numSamples++) {
        // Wait for magneto to become available
        while(!imu.isMagnetoAvailable()){
        }

        // Read magneto
        float magnetoData[3];
        imu.readMagneto(magnetoData);

        // Calculate heading angle from magneto data and set atomic value
        heading = atan2(magnetoData[0], magnetoData[2]);
    }
}

void opticalFlowThreadFunc(int cameraDevice, std::atomic<bool> &shouldQuit, std::atomic<float> &speed, unsigned int &numFrames)
{
    const float tau = 10.0f;
    const cv::Size unwrapRes(90, 30);

#ifdef USE_SEE3CAM
    const std::string deviceString = "/dev/video" + std::to_string(cameraDevice);
    See3CAM_CU40 cam(deviceString, See3CAM_CU40::Resolution::_672x380);

    // Create unwrapper to unwrap camera output
    const cv::Size camRes = cam.getOutputSize();
    auto unwrapper = cam.createUnwrapper(unwrapRes);
#else
    // Open video capture device and check it matches desired camera resolution
    cv::VideoCapture capture(cameraDevice);

    const cv::Size camRes(640, 480);
    BOB_ASSERT(capture.get(cv::CAP_PROP_FRAME_WIDTH) == camRes.width);
    BOB_ASSERT(capture.get(cv::CAP_PROP_FRAME_HEIGHT) == camRes.height);

    // Create unwrapper
    OpenCVUnwrap360 unwrapper(camRes, unwrapRes,
                              0.5, 0.416, 0.173, 0.377, -180_deg);
#endif

    // Create optical flow calculator
    OpenCVOpticalFlow opticalFlow(unwrapRes);

       // Create images
#ifdef USE_SEE3CAM
    cv::Mat greyscaleInput(camRes, CV_8UC1);
#else
    cv::Mat rgbInput(camRes, CV_8UC3);
    cv::Mat greyscaleInput(camRes, CV_8UC1);
#endif
    cv::Mat outputImage(unwrapRes, CV_8UC1);

    // Build a velocity filter whose preferred angle is going straight (0 degrees)
    cv::Mat velocityFilter(1, unwrapRes.width, CV_32FC1);
    buildOpticalFlowFilter(velocityFilter, 0.0f);

    cv::Mat flowXSum(1, unwrapRes.width, CV_32FC1);
    cv::Mat flowSum(1, 1, CV_32FC1);

    // Calculate filter coefficient
    const float alpha = 1.0f - std::exp(-1.0f / tau);

    // While quit signal isn't set
    float prevSpeed = 0.0f;
    for(numFrames = 0; !shouldQuit; numFrames++) {
#ifdef USE_SEE3CAM
        // Read directly into greyscale
        if(!cam.captureSuperPixelGreyscale(greyscaleInput)) {
            return;
        }
#else
        // Read from camera and convert to greyscale
        if(!capture.read(rgbInput)) {
            return;
        }
        cv::cvtColor(rgbInput, greyscaleInput, cv::COLOR_BGR2GRAY);
#endif

        // Unwrap
        unwrapper.unwrap(greyscaleInput, outputImage);

        // Calculate optical flow
        if(opticalFlow.calculate(outputImage)) {
            // Reduce horizontal flow - summing along columns
            cv::reduce(opticalFlow.getFlowX(), flowXSum, 0, cv::REDUCE_SUM);

            // Multiply summed flow by filters
            cv::multiply(flowXSum, velocityFilter, flowXSum);

            // Reduce filtered flow - summing along rows
            cv::reduce(flowXSum, flowSum, 1, cv::REDUCE_SUM);

            // Filter speed and set atomic
            prevSpeed = (alpha * flowSum.at<float>(0, 0)) + ((1.0f - alpha) * prevSpeed);
            speed = prevSpeed;
        }
    }

}
}   // Anonymous namespace


int main(int argc, char *argv[])
{
    const float velocityScale = 1.0f / 30.0f;

    // Create joystick interface
    Joystick joystick;

    // Create motor interface
    TANK_TYPE motor;

    // Initialise GeNN
    allocateMem();
    initialize();
    initializeSparse();

    // Atomic flag for quitting child threads
    std::atomic<bool> shouldQuit{false};

    // Create thread to read from IMU
    unsigned int numIMUSamples = 0;
    std::atomic<float> imuHeading{0.0f};
    std::thread imuThread(&imuThreadFunc,
                          std::ref(shouldQuit), std::ref(imuHeading), std::ref(numIMUSamples));

    // Create thread to calculate optical flow from camera device
    unsigned int numCameraFrames = 0;
    std::atomic<float> opticalFlowSpeed{0.0f};
    std::thread opticalFlowThread(&opticalFlowThreadFunc, (argc > 1) ? std::atoi(argv[1]) : 0,
                                  std::ref(shouldQuit), std::ref(opticalFlowSpeed), std::ref(numCameraFrames));

#ifdef RECORD_SENSORS
    std::ofstream data("data.csv");
#endif
    // Loop until second joystick button is pressed
    bool outbound = true;
    unsigned int numTicks = 0;
    unsigned int numOverflowTicks = 0;
    int64_t totalMicroseconds = 0;
    bool ignoreFlow = false;
    for(;; numTicks++) {
        // Record time at start of tick
        const auto tickStartTime = std::chrono::high_resolution_clock::now();

        // Read from joystick
        joystick.update();

        // Stop if 2nd button is pressed
        if(joystick.isDown(JButton::B)) {
            break;
        }

        // Update heading from IMU
        headingAngleTL = imuHeading;

        // Update speed from IMU
        // **NOTE** robot is incapable of holonomic motion!
        const float flow =  ignoreFlow ? 0.0f : (opticalFlowSpeed * velocityScale);
        speedTN2[Parameters::HemisphereLeft] = speedTN2[Parameters::HemisphereRight] = flow;

        // Push inputs to device
        pushspeedTN2ToDevice();

#ifdef RECORD_SENSORS
        data << imuHeading << ", " << flow << std::endl;
#endif
        // Step network
        stepTime();

        // Pull outputs from device
        pullrCPU4FromDevice();
        pullrCPU1FromDevice();

        // If we are going outbound
        if(outbound) {
            // Use joystick to drive motor
            motor.drive(joystick, RobotParameters::joystickDeadzone);

            // If first button is pressed switch to returning home
            if(joystick.isDown(JButton::A)) {
                LOGI << "Max CPU4 level r=" << *std::max_element(&rCPU4[0], &rCPU4[Parameters::numCPU4]) << ", i=" << *std::max_element(&iCPU4[0], &iCPU4[Parameters::numCPU4]);
                LOGI << "Returning home!";
                outbound = false;
            }

            ignoreFlow = (fabs(joystick.getState(JAxis::LeftStickHorizontal)) > fabs(joystick.getState(JAxis::LeftStickVertical)));
        }
        // Otherwise we're returning home so use CPU1 neurons to drive motor
        else {
            const float steer = driveMotorFromCPU1(motor, (numTicks % 100) == 0);
            ignoreFlow = (steer > 0.5f);
        }

        // Record time at end of tick
        const auto tickEndTime = std::chrono::high_resolution_clock::now();

        // Calculate tick duration (in microseconds)
        const int64_t tickMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(tickEndTime - tickStartTime).count();

        // Add to total
        totalMicroseconds += tickMicroseconds;

        // If there is time left in tick, sleep for remainder
        if(tickMicroseconds < RobotParameters::targetTickMicroseconds) {
            std::this_thread::sleep_for(std::chrono::microseconds(RobotParameters::targetTickMicroseconds - tickMicroseconds));
        }
        // Otherwise, increment overflow counter
        else {
            numOverflowTicks++;
        }
    }

    // Set quit flag and wait for child threads to complete
    shouldQuit = true;
    imuThread.join();
    opticalFlowThread.join();

    // Show stats
    LOGI << numOverflowTicks << "/" << numTicks << " ticks overflowed, mean tick time: " << (double)totalMicroseconds / (double)numTicks << "uS, ";
    LOGI << "IMU samples: " << numIMUSamples << ", ";
    LOGI << "Camera frames: " << numCameraFrames;

    // Stop motor
    motor.tank(0.0f, 0.0f);

    // Exit
    return EXIT_SUCCESS;
}
