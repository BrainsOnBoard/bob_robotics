// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/bn055_imu.h"
#include "common/lm9ds1_imu.h"
#include "plog/Log.h"
#include "common/timer.h"
#include "hid/joystick.h"
#include "net/imu_netsource.h"
#include "net/server.h"
#include "robots/mecanum.h"
#include "robots/norbot.h"
#include "robots/tank.h"
#include "third_party/units.h"
#include "vicon/capture_control.h"
#include "vicon/udp.h"
#include "video/netsink.h"

#include "os/keycodes.h"

#include <opencv2/opencv.hpp>

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
    virtual radian_t getHeading() = 0;
};

//---------------------------------------------------------------------------
// SpeedSource
//---------------------------------------------------------------------------
//! Interface for ways of getting velocity
class SpeedSource
{
public:
    virtual std::array<float, 2> getSpeed() = 0;
};

//---------------------------------------------------------------------------
// MotorController
//---------------------------------------------------------------------------
//! Interface to drive robot from model output
class MotorController
{
public:
    virtual void drive(float left, float right, bool log = false);
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
// HeadingSourceVicon
//---------------------------------------------------------------------------
class HeadingSourceVicon : public HeadingSource
{
public:
    HeadingSourceVicon(const Vicon::UDPClient<Vicon::ObjectDataVelocity> &vicon)
    :   m_Vicon(vicon)
    {
    }

    //------------------------------------------------------------------------
    // IMU virtuals
    //------------------------------------------------------------------------
    virtual radian_t getHeading() override
    {
        // Read data from VICON system
        auto objectData = m_Vicon.getObjectData();
        const auto &attitude = objectData.getPose().attitude();

        return -attitude[2];
    }

private:
    const Vicon::UDPClient<Vicon::ObjectDataVelocity> &m_Vicon;
};

//---------------------------------------------------------------------------
// SpeedSourceDeadReckon
//---------------------------------------------------------------------------
class SpeedSourceTankDeadReckon : public SpeedSource
{
public:
    SpeedSourceTankDeadReckon(const Robots::Tank &tank, float velocityScale = 1.0f / 10.0f)
    :   m_Tank(tank), m_VelocityScale(velocityScale)
    {
    }

    //------------------------------------------------------------------------
    // SpeedSource virtuals
    //------------------------------------------------------------------------
    virtual std::array<float, 2> getSpeed() override
    {
        const float speed =  (m_Tank.getLeft() + m_Tank.getRight()) * m_VelocityScale;
        return {speed, speed};
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const Robots::Tank &m_Tank;
    const float m_VelocityScale;
};

//---------------------------------------------------------------------------
// SpeedSourceOmni2DDeadReckon
//---------------------------------------------------------------------------
class SpeedSourceOmni2DDeadReckon : public SpeedSource
{
public:
    SpeedSourceOmni2DDeadReckon(const Robots::Omni2D &omni,
                                 const std::array<radian_t, 2> &preferredAngleTN2 = {45_deg, -45_deg},
                                 float velocityScale = 1.0f)
    :   m_Omni(omni), m_PreferredAngleTN2(preferredAngleTN2), m_VelocityScale(velocityScale)
    {
    }

    //------------------------------------------------------------------------
    // SpeedSource virtuals
    //------------------------------------------------------------------------
    virtual std::array<float, 2> getSpeed() override
    {
        // **TODO** sideways awesome
        const float speed =  m_Omni.getForwards() * m_VelocityScale;
        return {speed, speed};
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const Robots::Omni2D &m_Omni;
    const std::array<radian_t, 2> m_PreferredAngleTN2;
    const float m_VelocityScale;
};

//---------------------------------------------------------------------------
// SpeedSourceVicon
//---------------------------------------------------------------------------
class SpeedSourceVicon : public SpeedSource
{
public:
    SpeedSourceVicon(const Vicon::UDPClient<Vicon::ObjectDataVelocity> &vicon,
                     const std::array<radian_t, 2> &preferredAngleTN2 = {45_deg, -45_deg},
                     float speedScale = 5.0f)
    :   m_Vicon(vicon), m_PreferredAngleTN2(preferredAngleTN2), m_SpeedScale(speedScale)
    {
    }

    //------------------------------------------------------------------------
    // SpeedSource virtuals
    //------------------------------------------------------------------------
    virtual std::array<float, 2> getSpeed() override
    {
        // Read data from VICON system
        auto objectData = m_Vicon.getObjectData();
        const auto &velocity = objectData.getVelocity();
        const auto &attitude = objectData.getPose().attitude();

        const radian_t heading = -attitude[2];

        std::array<float, 2> speedTN2;
        for(size_t j = 0; j < Parameters::numTN2; j++) {
            speedTN2[j] = (sin(heading + m_PreferredAngleTN2[j]) * m_SpeedScale * velocity[0].value()) +
                          (cos(heading + m_PreferredAngleTN2[j]) * m_SpeedScale * velocity[1].value());
        }
        return speedTN2;
    }

private:
    const Vicon::UDPClient<Vicon::ObjectDataVelocity> &m_Vicon;
    const std::array<radian_t, 2> m_PreferredAngleTN2;
    const float m_SpeedScale;
};

//---------------------------------------------------------------------------
// MotorControllerOmni2D
//---------------------------------------------------------------------------
class MotorControllerTank : public MotorController
{
public:
    MotorControllerTank(Robots::Tank &tank)
    :   m_Tank(tank)
    {
    }

    virtual void drive(float left, float right, bool log) override
    {
        // Calculate differential steering signal
        const float steering = left - right;
        if(log) {
            LOGI << "Steer:" << steering;
        }

        // Clamp motor input values to be between -1 and 1
        const float leftMotor = 1.0f + steering;
        const float rightMotor = 1.0f - steering;
        m_Tank.tank(std::max(-1.f, std::min(1.f, leftMotor)), std::max(-1.f, std::min(1.f, rightMotor)));
    }

private:
    Robots::Tank &m_Tank;
};

//---------------------------------------------------------------------------
// MotorControllerOmni2D
//---------------------------------------------------------------------------
class MotorControllerOmni2D : public MotorController
{
public:
    MotorControllerOmni2D(Robots::Omni2D &omni)
    :   m_Omni(omni)
    {
    }

    virtual void drive(float left, float right, bool log) override
    {
        const float length = (left * left) + (right * right);

        // Steer based on signal
        const float steering = (left - right) / length;
        if(log) {
            LOGI << "Steer:" << steering;
        }

        // Clamp absolute steering value to [0,1] and subtract from 1
        // So no forward  speed if we're turning fast
        const float forward = 1.0f - std::min(1.0f, std::fabs(steering));

        m_Omni.omni2D(forward * 0.5f, 0.0f, steering * 8.0f);
    }

private:
    Robots::Omni2D &m_Omni;
};

std::unique_ptr<MotorController> createMotorController(Robots::Tank &tank)
{
    return std::make_unique<MotorControllerTank>(tank);
}

std::unique_ptr<MotorController> createMotorController(Robots::Omni2D &omni)
{
    return std::make_unique<MotorControllerOmni2D>(omni);
}

std::unique_ptr<SpeedSource> createSpeedSource(Robots::Tank &tank)
{
    return std::make_unique<SpeedSourceTankDeadReckon>(tank);
}

std::unique_ptr<SpeedSource> createSpeedSource(Robots::Omni2D &omni)
{
    return std::make_unique<SpeedSourceOmni2DDeadReckon>(omni);
}

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
    Robots::ROBOT_TYPE motor;

    std::unique_ptr<SpeedSource> speedSource = createSpeedSource(motor);
    std::unique_ptr<MotorController> motorController = createMotorController(motor);

#ifdef ROBOT_TYPE_EV3
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
        const auto speed = speedSource->getSpeed();
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

#ifdef ROBOT_TYPE_EV3
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
            // Sum left and right motor activity and pass to motor controller
            const float leftMotor = std::accumulate(&rCPU1[0], &rCPU1[8], 0.0f);
            const float rightMotor = std::accumulate(&rCPU1[8], &rCPU1[16], 0.0f);
            motorController->drive(leftMotor, rightMotor, (numTicks % 100) == 0);
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
