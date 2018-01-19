// Standard C++ includes
#include <atomic>
#include <chrono>
#include <thread>

// OpenCV includes


// Common includes
#include "../common/joystick.h"
#include "../common/lm9ds1_imu.h"
#include "../common/motor_i2c.h"
#include "../common/opencv_optical_flow.h"
#include "../common/opencv_unwrap_360.h"
#include "../common/timer.h"


// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

// Model includes
#include "parameters.h"
#include "robotParameters.h"
#include "simulatorCommon.h"

//---------------------------------------------------------------------------
// Anonymous namespace
//---------------------------------------------------------------------------
namespace
{
void buildOpticalFlowFilter(cv::Mat &filter, float preferredAngle) {
    // Loop through columns
    for(unsigned int x = 0; x < filter.cols; x++) {
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
    const cv::Size cameraRes(640, 480);
    const cv::Size unwrapRes(90, 10);

    // Open video capture device and check it matches desired camera resolution
    cv::VideoCapture capture(cameraDevice);
    assert(capture.get(cv::CAP_PROP_FRAME_WIDTH) == cameraRes.width);
    assert(capture.get(cv::CAP_PROP_FRAME_HEIGHT) == cameraRes.height);

    // Create unwrapper
    OpenCVUnwrap360 unwrapper(cameraRes, unwrapRes,
                              0.5, 0.416, 0.173, 0.377, -Parameters::pi);

    // Create optical flow calculator
    OpenCVOpticalFlow opticalFlow(unwrapRes);

    // Create images
    cv::Mat originalImage(cameraRes, CV_8UC3);
    cv::Mat outputImage(unwrapRes, CV_8UC3);

    // Build a velocity filter whose preferred angle is going straight (0 degrees)
    cv::Mat velocityFilter(1, unwrapRes.width, CV_32FC1);
    buildOpticalFlowFilter(velocityFilter, 0.0f);

    cv::Mat flowXSum(1, unwrapRes.width, CV_32FC1);
    cv::Mat flowSum(1, 1, CV_32FC1);
    
    // Read frames until should quit
    while(!shouldQuit) {
        // Read from camera
        if(!capture.read(originalImage)) {
            std::cerr << "Cannot read from camera" << std::endl;
            continue;
        }

        // Unwrap
        unwrapper.unwrap(originalImage, outputImage);

        if(opticalFlow.calculate(outputImage)) {
            // Reduce horizontal flow - summing along columns
            cv::reduce(opticalFlow.getFlowX(), flowXSum, 0, CV_REDUCE_SUM);

            // Multiply summed flow by filters
            cv::multiply(flowXSum, velocityFilter, flowXSum);

            // Reduce filtered flow - summing along rows
            cv::reduce(flowXSum, flowSum, 1, CV_REDUCE_SUM);
    
            // Calculate speed and set atomic value
            speed = flowSum.at<float>(0, 0);
        }
    }
    
}
}   // Anonymous namespace


int main(int argc, char *argv[])
{
    const float velocityScale = 1.0f / 500.0f;
    
    // Create joystick interface
    Joystick joystick;
    
    // Create motor interface
    MotorI2C motor;
    
    // Initialise GeNN
    allocateMem();
    initialize();

    //---------------------------------------------------------------------------
    // Initialize neuron parameters
    //---------------------------------------------------------------------------
    // TL
    for(unsigned int i = 0; i < 8; i++) {
        preferredAngleTL[i] = preferredAngleTL[8 + i] = (Parameters::pi / 4.0) * (double)i;
    }

    //---------------------------------------------------------------------------
    // Build connectivity
    //---------------------------------------------------------------------------
    buildConnectivity();
    
    initstone_cx();
    
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
    
    // Loop until second joystick button is pressed
    bool outbound = true;
    unsigned int numTicks = 0;
    unsigned int numOverflowTicks = 0;
    int64_t totalMicroseconds = 0;
    for(;; numTicks++) {
        // Record time at start of tick
        const auto tickStartTime = std::chrono::high_resolution_clock::now();
        
        // Read from joystick
        joystick.read();
        
        // Stop if 2nd button is pressed
        if(joystick.isButtonDown(1)) {
            break;
        }
        
        // Update heading from IMU
        headingAngleTL = imuHeading;
        
        // Update speed from IMU
        // **NOTE** robot is incapable of holonomic motion!
        speedTN2[Parameters::HemisphereLeft] = speedTN2[Parameters::HemisphereRight] = (opticalFlowSpeed * velocityScale);
        
        // Step network
        stepTimeCPU();
        
        // If we are going outbound
        if(outbound) {
            // Use joystick to drive motor
            joystick.drive(motor, RobotParameters::joystickDeadzone);
            
            // If first button is pressed switch to returning home
            if(joystick.isButtonDown(0)) {
                std::cout << "Returning home!" << std::endl;
                outbound = false;
            }
        }
        // Otherwise we're returning home so use CPU1 neurons to drive motor
        else {
            driveMotorFromCPU1(motor, RobotParameters::motorSteerThreshold, (numTicks % 100) == 0);
        }
        
        // Record time at end of tick
        const auto tickEndTime = std::chrono::high_resolution_clock::now();
        
        // Calculate tick duration (in microseconds)
        const int64_t tickMicroseconds = std::chrono::duration_cast<chrono::microseconds>(tickEndTime - tickStartTime).count();
        
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
    std::cout << numOverflowTicks << "/" << numTicks << " ticks overflowed, mean tick time: " << (double)totalMicroseconds / (double)numTicks << "uS, ";
    std::cout << "IMU sample rate: " << (double)numIMUSamples / ((double)numTicks * DT * 0.001) << "Hz, ";
    std::cout << "Camera frame rate: " << (double)numCameraFrames / ((double)numTicks * DT * 0.001) << "FPS" << std::endl;
    
    // Stop motor
    motor.tank(0.0f, 0.0f);
    
    // Exit
    return EXIT_SUCCESS;
}