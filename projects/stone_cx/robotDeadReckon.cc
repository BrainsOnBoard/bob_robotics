// BoB robotics includes
#include "common/main.h"
#include "common/logging.h"
#include "common/timer.h"
#include "hid/joystick.h"
#include "net/server.h"
#include "robots/tank.h"

#ifdef USE_EV3
#include "net/imu_netsource.h"
#include "os/keycodes.h"

#include <opencv2/opencv.hpp>
#else
#include "common/lm9ds1_imu.h"
#include "video/netsink.h"
#endif

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

// Model includes
#include "parameters.h"
#include "robotCommon.h"
#include "robotParameters.h"
#include "visualizationCommon.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <thread>

using namespace BoBRobotics;
using namespace BoBRobotics::StoneCX;
using namespace BoBRobotics::HID;

//---------------------------------------------------------------------------
// Anonymous namespace
//---------------------------------------------------------------------------
namespace
{
#ifdef USE_EV3
float
getIMUHeading(Net::IMUNetSource &imu)
{
    using namespace units::angle;
    return static_cast<radian_t>(imu.getYaw()).value();
}
#else
float
getIMUHeading(LM9DS1 &imu)
{
    // Wait for magneto to become available
    while (!imu.isMagnetoAvailable()) {
    }

    // Read magneto
    float magnetoData[3];
    imu.readMagneto(magnetoData);

    // Calculate heading angle from magneto data and set atomic value
    return atan2(magnetoData[0], magnetoData[2]);
}
#endif

void imuThreadFunc(Net::Connection &connection,
                   std::atomic<bool> &shouldQuit,
                   std::atomic<float> &heading,
                   unsigned int &numSamples)
{
#ifndef USE_EV3
    (void) connection; // unused arg

    // Create IMU interface
    LM9DS1 imu;

    // Initialise IMU magnetometer
    LM9DS1::MagnetoSettings magSettings;
    imu.initMagneto(magSettings);
#else
    Net::IMUNetSource imu(connection);
#endif

    // While quit signal isn't set
    for (numSamples = 0; !shouldQuit; numSamples++) {
        heading = getIMUHeading(imu);
    }
}
}   // Anonymous namespace

int bob_main(int argc, char *argv[])
{
    // Simulation rendering parameters
    constexpr unsigned int activityImageWidth = 500;
    constexpr unsigned int activityImageHeight = 1000;
    constexpr float velocityScale = 1.f / 10.f;
    const bool doVisualise = (argc > 1) && strcmp(argv[1], "--visualise") == 0;

    // Create motor interface
    Robots::TANK_TYPE motor;

#ifdef USE_EV3
    Net::Connection *connection = &motor.getConnection();
#else
    std::unique_ptr<Net::Server> server;
    std::unique_ptr<Net::Connection> connection;
    std::unique_ptr<Video::NetSink> netSink;

    // If command line arguments are specified, run connection
    if(doVisualise) {
        LOGI << "Streaming activity over network";

        // Create server and sink for sending activity image over network
        server = std::make_unique<Net::Server>();
        connection = std::make_unique<Net::Connection>(server->waitForConnection());
        netSink = std::make_unique<Video::NetSink>(*connection,
                                                   cv::Size(activityImageWidth, activityImageHeight),
                                                   "activity");
        connection->runInBackground();
    }
#endif
    if (!doVisualise) {
        std::cout << "Use --visualise argument to visualise network" << std::endl;
    }

    // Create joystick interface
    Joystick joystick;

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
                          std::ref(*connection),
                          std::ref(shouldQuit),
                          std::ref(imuHeading),
                          std::ref(numIMUSamples));

    cv::Mat activityImage(activityImageHeight, activityImageWidth, CV_8UC3, cv::Scalar::all(0));

#ifdef RECORD_SENSORS
    std::ofstream data("data.csv");
#endif

    // Loop until second joystick button is pressed
    bool outbound = true;
    unsigned int numTicks = 0;
    unsigned int numOverflowTicks = 0;
    int64_t totalMicroseconds = 0;
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
        if(headingAngleTL < 0.0) {
            headingAngleTL = (2.0 * Parameters::pi) + headingAngleTL;
        }

        // Calculate dead reckoning speed from motor
        const float speed =  (motor.getLeft() + motor.getRight()) * velocityScale;
        speedTN2[Parameters::HemisphereLeft] = speedTN2[Parameters::HemisphereRight] = speed;

        // Push inputs to device
        pushspeedTN2ToDevice();

#ifdef RECORD_SENSORS
        data << imuHeading << ", " << speed;
#endif
        // Step network
        stepTime();

        // Pull outputs from device
        pullrCPU4FromDevice();
        pullrCPU1FromDevice();

        // If we should be streaming activity
        if(doVisualise) {
            // Pull additional outputs from device
            pullrTLFromDevice();
            pullrTN2FromDevice();
            pullrCL1FromDevice();
            pullrTB1FromDevice();
            pullrPontineFromDevice();

            // Render network activity
            visualize(activityImage);

#ifdef USE_EV3
            cv::imshow("activity", activityImage);
            if ((cv::waitKeyEx(1) & OS::KeyMask) == OS::KeyCodes::Escape) {
                break;
            }
#else
            // Send activity image
            netSink->sendFrame(activityImage);
#endif
        }

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

        }
        // Otherwise we're returning home so use CPU1 neurons to drive motor
        else {
            driveMotorFromCPU1(motor, (numTicks % 100) == 0);
            if(joystick.isDown(JButton::A)) {
                outbound = true;
            }
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

    // Show stats
    LOGI << numOverflowTicks << "/" << numTicks << " ticks overflowed, mean tick time: " << (double)totalMicroseconds / (double)numTicks << "uS, ";
    LOGI << "IMU samples: " << numIMUSamples << ", ";

    // Exit
    return EXIT_SUCCESS;
}
