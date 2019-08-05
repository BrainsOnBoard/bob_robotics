// Standard C++ includes
#include <chrono>
#include <fstream>
#include <limits>
#include <memory>

// Standard C includes
#include <cassert>

// BoB robotics includes
#include "common/fsm.h"
#include "common/logging.h"
#include "common/timer.h"
#include "hid/joystick.h"
#include "imgproc/opencv_unwrap_360.h"
#include "net/server.h"
#include "robots/norbot.h"
#include "vicon/capture_control.h"
#include "video/netsink.h"
#include "vicon/udp.h"
#include "video/panoramic.h"

// BoB robotics third-party includes
#include "third_party/path.h"

// Snapshot bot includes
#include "config.h"
#include "image_input.h"
#include "memory.h"

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::math;

//------------------------------------------------------------------------
// Anonymous namespace
//------------------------------------------------------------------------
namespace
{
enum class State
{
    Invalid,
    WaitToTrain,
    Training,
    WaitToTest,
    Testing,
};

//------------------------------------------------------------------------
// RobotFSM
//------------------------------------------------------------------------
class RobotFSM : FSM<State>::StateHandler
{
    using TimePoint = std::chrono::high_resolution_clock::time_point;
    using Seconds = std::chrono::duration<double, std::ratio<1>>;
    using Milliseconds = std::chrono::duration<double, std::milli>;

public:
    RobotFSM(const Config &config)
    :   m_Config(config), m_StateMachine(this, State::Invalid), m_Camera(Video::getPanoramicCamera()),
        m_Output(m_Camera->getOutputSize(), CV_8UC3), m_Unwrapped(config.getUnwrapRes(), CV_8UC3),
        m_DifferenceImage(config.getUnwrapRes(), CV_8UC1), m_Unwrapper(m_Camera->createUnwrapper(config.getUnwrapRes())),
        m_ImageInput(createImageInput(config)), m_Memory(createMemory(config, m_ImageInput->getOutputSize())),
        m_TestDuration(450.0),/*
        m_Server(config.getServerListenPort()), m_NetSink(m_Server, config.getUnwrapRes(), "unwrapped"),*/
        m_NumSnapshots(0)
    {
        // Create output directory (if necessary)
        filesystem::create_directory(m_Config.getOutputPath());

        // If we should stream output, run server thread
        /*if(m_Config.shouldStreamOutput()) {
            m_Server.runInBackground();
        }*/

        // If we should use Vicon tracking
        if(m_Config.shouldUseViconTracking()) {
            // Connect to port specified in config
            m_ViconTracking.connect(m_Config.getViconTrackingPort());
        }

        // If we should use Vicon capture control
        if(m_Config.shouldUseViconCaptureControl()) {
            // Connect to capture host system specified in config
            m_ViconCaptureControl.connect(m_Config.getViconCaptureControlHost(), m_Config.getViconCaptureControlPort(),
                                          m_Config.getViconCaptureControlPath());

            // Start capture
            m_ViconCaptureControl.startRecording(m_Config.getViconCaptureControlName());
        }

        // If we should train
        if(m_Config.shouldTrain()) {
            // Start in training state
            m_StateMachine.transition(State::WaitToTrain);
        }
        else {
            // If we're not using InfoMax or pre-trained weights don't exist
            if(!m_Config.shouldUseInfoMax() || !(m_Config.getOutputPath() / ("weights" + config.getTestingSuffix() + ".bin")).exists()) {
                LOGI << "Training on stored snapshots";
                for(m_NumSnapshots = 0;;m_NumSnapshots++) {
                    const auto filename = getSnapshotPath(m_NumSnapshots);

                    // If file exists, load image and train memory on it
                    if(filename.exists()) {
                        std::cout << "." << std::flush;
                        const auto &processedSnapshot = m_ImageInput->processSnapshot(cv::imread(filename.str()));
                        //cv::imwrite((m_Config.getOutputPath() / ("processed_" + std::to_string(m_NumSnapshots) + ".png")).str(), processedSnapshot);
                        m_Memory->train(processedSnapshot);
                    }
                    // Otherwise, stop searching
                    else {
                        break;
                    }
                }
                LOGI << "Loaded " << m_NumSnapshots << " snapshots";

                // If we are using InfoMax save the weights now
                if(m_Config.shouldUseInfoMax()) {
                    InfoMax *infoMax= dynamic_cast<InfoMax*>(m_Memory.get());
                    infoMax->saveWeights((m_Config.getOutputPath() / "weights.bin").str());
                }
            }

            // Start directly in testing state
            m_StateMachine.transition(State::WaitToTest);
        }
    }

    ~RobotFSM()
    {
        // Stop motors
        m_Motor.tank(0.0f, 0.0f);
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool update()
    {
        return m_StateMachine.update();
    }

private:
    filesystem::path getSnapshotPath(size_t index) const
    {
        return m_Config.getOutputPath() / ("snapshot_" + std::to_string(index) + ".png");
    }

    //------------------------------------------------------------------------
    // FSM::StateHandler virtuals
    //------------------------------------------------------------------------
    virtual bool handleEvent(State state, Event event) override
    {
        // If this event is an update
        if(event == Event::Update) {
            // Read joystick
            m_Joystick.update();

            // Exit if X is pressed
            if(m_Joystick.isPressed(HID::JButton::X)) {
                 // If we should use Vicon capture control
                if(m_Config.shouldUseViconCaptureControl()) {
                    // Stop capture
                    m_ViconCaptureControl.stopRecording(m_Config.getViconCaptureControlName());
                }
                return false;
            }

            // Capture frame
            if(!m_Camera->readFrame(m_Output)) {
                return false;
            }

            // Unwrap frame
            m_Unwrapper.unwrap(m_Output, m_Unwrapped);

            cv::waitKey(1);
        }

        if(state == State::WaitToTrain) {
            if(event == Event::Enter) {
                LOGI << "Press B to start training" ;
            }
            else if(event == Event::Update) {
                if(m_Joystick.isPressed(HID::JButton::B)) {
                    m_StateMachine.transition(State::Training);
                }
            }
        }
        else if(state == State::Training) {
            if(event == Event::Enter) {
                LOGI << "Starting training";

                // Open settings file and write unwrapper settings to it
                cv::FileStorage settingsFile((m_Config.getOutputPath() / "training_settings.yaml").str(), cv::FileStorage::WRITE);
                settingsFile << "unwrapper" << m_Unwrapper;

                // Close log file if it's already open
                if(m_LogFile.is_open()) {
                    m_LogFile.close();
                }

                m_LogFile.open((m_Config.getOutputPath() / "training.csv").str());
                BOB_ASSERT(m_LogFile.good());

                // Write header
                m_LogFile << "Time [s], Filename";

                // If Vicon tracking is available, write additional header
                if(m_Config.shouldUseViconTracking()) {
                    m_LogFile << ", Frame, X, Y, Z, Rx, Ry, Rz";
                }
                m_LogFile << std::endl;

                // Reset train time and test image
                m_LastTrainTime = m_RecordingStartTime = std::chrono::high_resolution_clock::now();

                // Delete old snapshots
                const std::string snapshotWildcard = (m_Config.getOutputPath() / "snapshot_*.png").str();
                system(("rm -f " + snapshotWildcard).c_str());
            }
            else if(event == Event::Update) {
                const auto currentTime = std::chrono::high_resolution_clock::now();

                // While testing, if we should stream output, send unwrapped frame
                /*if(m_Config.shouldStreamOutput()) {
                    m_NetSink.sendFrame(m_Unwrapped);
                }*/

                // Drive motors using joystick
                m_Motor.drive(m_Joystick, m_Config.getJoystickDeadzone());

                // If A is pressed
                if(m_Joystick.isPressed(HID::JButton::A) || (m_Config.shouldAutoTrain() && (currentTime - m_LastTrainTime) > m_Config.getTrainInterval())) {
                    // Update last train time
                    m_LastTrainTime = currentTime;

                    // Train memory
                    LOGI << "\tTrained snapshot" ;
                    m_Memory->train(m_ImageInput->processSnapshot(m_Unwrapped));

                    // Write raw snapshot to disk
                    const std::string filename = getSnapshotPath(m_NumSnapshots++).str();
                    cv::imwrite(filename, m_Unwrapped);

                    // Write time
                    m_LogFile << ((Seconds)(currentTime - m_RecordingStartTime)).count() << ", " << filename;

                    // If Vicon tracking is available
                    if(m_Config.shouldUseViconTracking()) {
                        // Get tracking data
                        auto objectData = m_ViconTracking.getObjectData(m_Config.getViconTrackingObjectName());
                        const auto &position = objectData.getPosition<units::length::millimeter_t>();
                        const auto &attitude = objectData.getAttitude<units::angle::degree_t>();

                        // Write to CSV
                        m_LogFile << ", " << objectData.getFrameNumber() << ", " << position[0].value() << ", " << position[1].value() << ", " << position[2].value() << ", " << attitude[0].value() << ", " << attitude[1].value() << ", " << attitude[2].value();
                    }
                    m_LogFile << std::endl;
                }
                // Otherwise, if B is pressed, go to testing
                else if(m_Joystick.isPressed(HID::JButton::B)) {
                    m_StateMachine.transition(State::WaitToTest);
                }
            }
        }
        else if(state == State::WaitToTest) {
            if(event == Event::Enter) {
                LOGI << "Press B to start testing" ;
            }
            else if(event == Event::Update) {
                if(m_Joystick.isPressed(HID::JButton::B)) {
                    m_StateMachine.transition(State::Testing);
                }
            }
        }
        else if(state == State::Testing) {
            if(event == Event::Enter) {
                LOGI << "Testing: finding snapshot" ;

                // Open settings file and write unwrapper settings to it
                cv::FileStorage settingsFile((m_Config.getOutputPath() / "testing_settings.yaml").str().c_str(), cv::FileStorage::WRITE);
                settingsFile << "unwrapper" << m_Unwrapper;

                // Close log file if it's already open
                if(m_LogFile.is_open()) {
                    m_LogFile.close();
                }

                // Open log file
                m_LogFile.open((m_Config.getOutputPath() / ("testing" + m_Config.getTestingSuffix() + ".csv")).str());
                BOB_ASSERT(m_LogFile.good());

                // Write heading for time column
                m_LogFile << "Time [s], ";

                // Write memory-specific CSV header
                m_Memory->writeCSVHeader(m_LogFile);

                // If Vicon tracking is available, write additional header fields
                if(m_Config.shouldUseViconTracking()) {
                    m_LogFile << ", Frame number, X, Y, Z, Rx, Ry, Rz";
                }

                if(m_Config.shouldSaveTestingDiagnostic()) {
                    m_LogFile << ", Filename";
                }
                m_LogFile << std::endl;

                // Reset test time and test image
                m_LastMotorCommandTime = m_RecordingStartTime = std::chrono::high_resolution_clock::now();
                m_TestImageIndex = 0;

                // Delete old testing images
                const std::string testWildcard = (m_Config.getOutputPath() / ("test" +  m_Config.getTestingSuffix() + "_*.png")).str();
                system(("rm -f " + testWildcard).c_str());
            }
            else if(event == Event::Update) {
                const auto currentTime = std::chrono::high_resolution_clock::now();

                // If it's time to move
                if((currentTime - (m_LastMotorCommandTime + m_TestDuration)) > m_Config.getMotorCommandInterval()) {
                    // Find matching snapshot
                    m_Memory->test(m_ImageInput->processSnapshot(m_Unwrapped));

                    // Write time
                    m_LogFile << ((Seconds)(currentTime - m_RecordingStartTime)).count() << ", ";

                    // Write memory-specific CSV logging
                    m_Memory->writeCSVLine(m_LogFile);

                    // If vicon tracking is available
                    if(m_Config.shouldUseViconTracking()) {
                        // Get tracking data
                        auto objectData = m_ViconTracking.getObjectData(0);
                        const auto &position = objectData.getPosition<units::length::millimeter_t>();
                        const auto &attitude = objectData.getAttitude<units::angle::degree_t>();

                        // Write extra logging data
                        m_LogFile << ", " << objectData.getFrameNumber() << ", " << position[0] << ", " << position[1] << ", " << position[2] << ", " << attitude[0] << ", " << attitude[1] << ", " << attitude[2];
                    }

                    // If we should save diagnostics when testing
                    if(m_Config.shouldSaveTestingDiagnostic()) {
                        const std::string filename = "test" + m_Config.getTestingSuffix() + "_" + std::to_string(m_TestImageIndex++) + ".png";
                        m_LogFile << ", " << filename;
                        // Build path to test image and save
                        const auto testImagePath = m_Config.getOutputPath() / filename;
                        cv::imwrite(testImagePath.str(), m_Unwrapped);
                    }

                    m_LogFile << std::endl;

                    // If we should stream output
                    /*if(m_Config.shouldStreamOutput()) {
                        // Attempt to dynamic cast memory to a perfect memory
                        PerfectMemory *perfectMemory = dynamic_cast<PerfectMemory*>(m_Memory.get());
                        if(perfectMemory != nullptr) {
                            // Get matched snapshot
                            const cv::Mat &matchedSnapshot = perfectMemory->getBestSnapshot();

                            // Calculate difference image
                            cv::absdiff(matchedSnapshot, m_Unwrapped, m_DifferenceImage);

                            char status[255];
                            sprintf(status, "Angle:%f deg, Min difference:%f", degree_t(perfectMemory->getBestHeading()).value(), perfectMemory->getLowestDifference());
                            cv::putText(m_DifferenceImage, status, cv::Point(0, m_Config.getUnwrapRes().height -20),
                                        cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, 0xFF);

                            // Send annotated difference image
                            //m_NetSink.sendFrame(m_DifferenceImage);
                        }
                        else {
                            LOGW << "WARNING: Can only stream output from a perfect memory";
                        }
                    }*/
                    // Get time after testing and thus calculate how long it took
                    const auto motorTime = std::chrono::high_resolution_clock::now();
                    m_TestDuration = motorTime - currentTime;

                    // If test duration is longer than motor command interval, this schedule cannot be maintained so give a warning
                    if(m_TestDuration > m_Config.getMotorCommandInterval()) {
                        LOGW << "last test took " << m_TestDuration.count() << "ms - this is longer than desired motor command interval (" << m_Config.getMotorCommandInterval().count() << "ms)";
                    }

                    // Reset move time
                    m_LastMotorCommandTime = motorTime;

                    // Determine how fast we should turn based on the absolute angle
                    auto turnSpeed = m_Config.getTurnSpeed(m_Memory->getBestHeading());

                    // If we should turn, do so
                    if(turnSpeed > 0.0f) {
                        const float motorTurn = (m_Memory->getBestHeading() <  0.0_deg) ? turnSpeed : -turnSpeed;
                        m_Motor.tank(motorTurn, -motorTurn);
                    }
                    // Otherwise drive forwards
                    else {
                        m_Motor.tank(m_Config.getMoveSpeed(), m_Config.getMoveSpeed());
                    }
                }
            }
        }
        else {
            LOGE << "Invalid state";
            return false;
        }
        return true;
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Configuration
    const Config &m_Config;

    // State machine
    FSM<State> m_StateMachine;

    // Camera interface
    std::unique_ptr<Video::Input> m_Camera;

    // Joystick interface
    HID::Joystick m_Joystick;

    // OpenCV images used to store raw camera frame and unwrapped panorama
    cv::Mat m_Output;
    cv::Mat m_Unwrapped;
    cv::Mat m_DifferenceImage;

    // OpenCV-based panorama unwrapper
    ImgProc::OpenCVUnwrap360 m_Unwrapper;

    // Image processor
    std::unique_ptr<ImageInput> m_ImageInput;

    // Perfect memory
    std::unique_ptr<MemoryBase> m_Memory;

    // Motor driver
    Robots::Norbot m_Motor;

    // Last time at which a motor command was issued or a snapshot was trained
    TimePoint m_LastMotorCommandTime;
    TimePoint m_LastTrainTime;

    // Time at which testing or training started
    TimePoint m_RecordingStartTime;

    Milliseconds m_TestDuration;

    // Index of test image to write
    size_t m_TestImageIndex;

    // Vicon tracking interface
    Vicon::UDPClient<Vicon::ObjectData> m_ViconTracking;

    // Vicon capture control interface
    Vicon::CaptureControl m_ViconCaptureControl;

    // CSV file containing logging
    std::ofstream m_LogFile;

    // How many snapshots has memory been trained on
    size_t m_NumSnapshots;

    // Server for streaming etc
    //Net::Server m_Server;

    // Sink for video to send over server
    //Video::NetSink m_NetSink;
};
}   // Anonymous namespace

int main(int argc, char *argv[])
{
    const char *configFilename = (argc > 1) ? argv[1] : "config.yaml";

    // Read config values from file
    Config config;
    {
        cv::FileStorage configFile(configFilename, cv::FileStorage::READ);
        if(configFile.isOpened()) {
            configFile["config"] >> config;
        }
    }

    // Re-write config file
    {
        cv::FileStorage configFile(configFilename, cv::FileStorage::WRITE);
        configFile << "config" << config;
    }

    RobotFSM robot(config);

    {
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0; robot.update(); frame++) {
        }

        const double msPerFrame = timer.get() / (double)frame;
        LOGI << "FPS:" << 1000.0 / msPerFrame;
    }

    return 0;
}
