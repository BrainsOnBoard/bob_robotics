// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/bn055_imu.h"
#include "common/lm9ds1_imu.h"
#include "common/timer.h"
#include "hid/joystick.h"
#include "hid/robot_control.h"
#include "net/imu_netsource.h"
#include "net/server.h"
#include "os/keycodes.h"
#include "robots/robot_type.h"
#include "video/netsink.h"

// Third-party includes
#include "plog/Log.h"

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

// Model includes
#include "parameters.h"
#include "robotParameters.h"
#include "visualizationCommon.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <numeric>
#include <thread>

using namespace units::literals;
using namespace units::math;
using namespace units::angle;

using namespace BoBRobotics;
using namespace BoBRobotics::StoneCX;
using namespace BoBRobotics::HID;

//---------------------------------------------------------------------------
// Anonymous namespace
//---------------------------------------------------------------------------
namespace
{

//---------------------------------------------------------------------------
// HeadingSource
//---------------------------------------------------------------------------
//! Interface for ways of getting heading
class HeadingSource
{
public:
    virtual ~HeadingSource() = default;
    virtual radian_t getHeading() = 0;
};

//---------------------------------------------------------------------------
// HeadingSourceLM9DS1IMU
//---------------------------------------------------------------------------
class HeadingSourceLM9DS1IMU : public HeadingSource
{
public:
    HeadingSourceLM9DS1IMU()
    {
        // Initialise IMU magnetometer was default settings
        LM9DS1::MagnetoSettings magSettings;
        m_IMU.initMagneto(magSettings);
    }

    virtual radian_t getHeading() override
    {
        // Wait for magneto to become available
        while(!m_IMU.isMagnetoAvailable()){
        }

        // Read magneto
        float magnetoData[3];
        m_IMU.readMagneto(magnetoData);

        // Calculate heading angle from magneto data and set atomic value
        return radian_t(std::atan2(magnetoData[0], magnetoData[2]));
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    LM9DS1 m_IMU;
};

//---------------------------------------------------------------------------
// HeadingSourceBN055IMU
//---------------------------------------------------------------------------
class HeadingSourceBN055IMU : public HeadingSource
{
public:
    virtual radian_t getHeading() override
    {
        return m_IMU.getEulerAngles()[0];
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    BN055 m_IMU;
};

//---------------------------------------------------------------------------
// IMUEV3
//---------------------------------------------------------------------------
class HeadingSourceEV3IMU : public HeadingSource
{
public:
    HeadingSourceEV3IMU(Net::Connection &connection)
    :   m_NetSource(connection)
    {
    }

    virtual radian_t getHeading() override
    {
        using namespace units::angle;
        return static_cast<radian_t>(m_NetSource.getYaw());
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    Net::IMUNetSource m_NetSource;
};

//---------------------------------------------------------------------------
// MotorController
//---------------------------------------------------------------------------
class MotorController
{
public:
    MotorController(ROBOT_TYPE &robot)
      : m_Robot(robot)
    {}

    void drive(float left, float right, bool log)
    {
#ifdef ROBOT_TYPE_OMNI2D_MECANUM
        const float length = (left * left) + (right * right);

        // Steer based on signal
        const float steering = (left - right) / length;
        if(log) {
            LOGI << "Steer:" << steering;
        }

        // Clamp absolute steering value to [0,1] and subtract from 1
        // So no forward  speed if we're turning fast
        m_Forward = 0.5f * (1.0f - std::min(1.0f, std::fabs(steering)));
        m_Robot.omni2D(m_Forward, 0.0f, steering * 8.0f);
#else
        // Calculate differential steering signal
        const float steering = left - right;
        if(log) {
            LOGI << "Steer:" << steering;
        }

        // Clamp motor input values to be between -1 and 1
        const float leftMotor = std::max(-1.f, std::min(1.f, 1.0f + steering));
        const float rightMotor = std::max(-1.f, std::min(1.f, 1.0f - steering));
        m_Robot.tank(leftMotor, rightMotor);

        m_Forward = leftMotor + rightMotor;
#endif
    }

    std::array<float, 2> getSpeed() const
    {
#ifdef ROBOT_TYPE_OMNI2D_MECANUM
        constexpr float VelocityScale = 1.f;

        // **TODO** sideways awesome
#else
        constexpr float VelocityScale = 0.1f;
#endif

        const float speed = m_Forward * VelocityScale;

        return { speed, speed };
    }

private:
    ROBOT_TYPE &m_Robot;
    float m_Forward = 0.f;
};

void readHeadingThreadFunc(HeadingSource *headingSource,
                           std::atomic<bool> &shouldQuit,
                           std::atomic<float> &heading,
                           unsigned int &numSamples)
{
    // While quit signal isn't set
    for (numSamples = 0; !shouldQuit; numSamples++) {
        heading = headingSource->getHeading().value();
    }
}
}   // Anonymous namespace

int bobMain(int argc, char *argv[])
{
    // Simulation rendering parameters
    constexpr unsigned int activityImageWidth = 500;
    constexpr unsigned int activityImageHeight = 1000;
    const bool doVisualise = (argc > 1) && strcmp(argv[1], "--visualise") == 0;

    // Create motor interface
    ROBOT_TYPE motor;

    MotorController motorController{ motor };

#ifdef ROBOT_TYPE_EV3_EV3
    std::unique_ptr<HeadingSource> headingSource = std::make_unique<HeadingSourceEV3IMU>(motor.getConnection());
#else
    std::unique_ptr<Net::Server> server;
    std::unique_ptr<Net::Connection> connection;
    std::unique_ptr<Video::NetSink> netSink;
    std::unique_ptr<HeadingSource> headingSource = std::make_unique<HeadingSourceBN055IMU>();

    // If command line arguments are specified, run connection
    if(doVisualise) {
        LOGI << "Streaming activity over network";

        // Create server and sink for sending activity image over network
        server = std::make_unique<Net::Server>();
        connection = server->waitForConnection();
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
    unsigned int numHeadingSamples = 0;
    std::atomic<float> heading{0};
    std::thread readHeadingThread(&readHeadingThreadFunc, headingSource.get(), std::ref(shouldQuit),
                                  std::ref(heading), std::ref(numHeadingSamples));

    cv::Mat activityImage(activityImageHeight, activityImageWidth, CV_8UC3, cv::Scalar::all(0));

#ifdef RECORD_SENSORS
    std::ofstream data("data.csv");
    data.exceptions(std::ios::badbit | std::ios::failbit);
#endif

    // Run server in background,, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C

    // Loop until second joystick button is pressed
    bool outbound = true;
    unsigned int numTicks = 0;
    unsigned int numOverflowTicks = 0;
    int64_t totalMicroseconds = 0;
    for(;; numTicks++) {
        // Record time at start of tick
        const auto tickStartTime = std::chrono::high_resolution_clock::now();

        // Retrow any exceptions caught on background thread
        catcher.check();

        // Read from joystick
        joystick.update();

        // Stop if 2nd button is pressed
        if(joystick.isDown(JButton::B)) {
            break;
        }

        // Update heading from IMU
        headingAngleTL = heading;
        if(headingAngleTL < 0.0) {
            headingAngleTL = (2.0 * Parameters::pi) + headingAngleTL;
        }

        // Calculate dead reckoning speed from motor
        const auto speed = motorController.getSpeed();
        speedTN2[Parameters::HemisphereLeft] = speed[0];
        speedTN2[Parameters::HemisphereRight] = speed[1];

        // Push inputs to device
        pushspeedTN2ToDevice();

#ifdef RECORD_SENSORS
        data << heading << ", " << speed;
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

#ifdef ROBOT_TYPE_EV3_EV3
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
            HID::drive(motor, joystick, RobotParameters::joystickDeadzone);

            // If first button is pressed switch to returning home
            if(joystick.isDown(JButton::A)) {
                LOGI << "Max CPU4 level r=" << *std::max_element(&rCPU4[0], &rCPU4[Parameters::numCPU4]) << ", i=" << *std::max_element(&iCPU4[0], &iCPU4[Parameters::numCPU4]);
                LOGI << "Returning home!";
                outbound = false;
            }

        }
        // Otherwise we're returning home so use CPU1 neurons to drive motor
        else {
            // Sum left and right motor activity and pass to motor controller
            const float leftMotor = std::accumulate(&rCPU1[0], &rCPU1[8], 0.0f);
            const float rightMotor = std::accumulate(&rCPU1[8], &rCPU1[16], 0.0f);
            motorController.drive(leftMotor, rightMotor, (numTicks % 100) == 0);
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
    readHeadingThread.join();

    // Show stats
    LOGI << numOverflowTicks << "/" << numTicks << " ticks overflowed, mean tick time: " << (double)totalMicroseconds / (double)numTicks << "uS, ";
    LOGI << "IMU samples: " << numHeadingSamples << ", ";

    // Exit
    return EXIT_SUCCESS;
}
