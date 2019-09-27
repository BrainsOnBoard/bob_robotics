// BoB robotics includes
#include "common/fsm.h"
#include "common/logging.h"
#include "common/pose_getter.h"
#include "common/timer.h"
#include "hid/joystick_sfml_keyboard.h"
#include "imgproc/opencv_unwrap_360.h"
#include "robots/tank.h"
#include "vicon/capture_control.h"
#include "video/netsink.h"
#include "viz/sfml_world/sfml_world.h"

#ifdef LOCAL_DISPLAY
#include "os/keycodes.h"
#else
#include "net/server.h"
#endif

// Third-party includes
#include "third_party/path.h"

// Snapshot bot includes
#include "config.h"
#include "image_input.h"
#include "memory.h"

// Standard C++ includes
#include <chrono>
#include <fstream>
#include <limits>
#include <memory>
#include <utility>

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

using Bounds = std::pair<Vector3<meter_t>, Vector3<meter_t>>;

#if POSE_GETTER_TYPE_ANTWORLD_TANK
auto getMinBounds(POSE_GETTER_TYPE &poseGetter)
{
    return poseGetter->getMinBound();
}

auto getMaxBounds(POSE_GETTER_TYPE &poseGetter)
{
    return poseGetter->getMaxBound();
}
#else
auto getMinBounds(POSE_GETTER_TYPE &)
{
    // Roughly the dimensions of Sussex's Vicon arena
    constexpr auto limit = -3.2_m;
    return Vector3<meter_t>{limit, limit, limit};
}

auto getMaxBounds(POSE_GETTER_TYPE &)
{
    constexpr auto limit = 3.2_m;
    return Vector3<meter_t>{limit, limit, limit};
}
#endif

//----------------------------------- Tank-------------------------------------
// RobotFSM
//------------------------------------------------------------------------
class RobotFSM : FSM<State>::StateHandler
{
    using TimePoint = std::chrono::high_resolution_clock::time_point;
    using Seconds = std::chrono::duration<double, std::ratio<1>>;
    using Milliseconds = std::chrono::duration<double, std::milli>;

public:
    RobotFSM(const Config &config)
    :   m_Config(config),
        m_StateMachine(this, State::Invalid),
        m_Motor{},
        m_PoseGetter(createPoseGetter(m_Motor)),
        m_Camera(m_Motor.getCamera()),
        m_Display(getMinBounds(m_PoseGetter), getMaxBounds(m_PoseGetter)),
        m_CarSprite(m_Display, 104_mm),
        m_TrainPath(m_Display.createLineStrip(sf::Color::Green)),
        m_TestPath(m_Display.createLineStrip(sf::Color::Blue)),
        m_Joystick(HID::JoystickSFMLKeyboard::createJoystick(m_Display.getWindow())),
        m_Unwrapped(config.getUnwrapRes(), CV_8UC3),
        m_ImageInput(createImageInput(config)),
        m_Memory(createMemory(config, m_ImageInput->getOutputSize())),
        m_TestDuration(450.0),
        m_NumSnapshots(0)
    {
        LOGI << "Min bounds: " << getMinBounds(m_PoseGetter);
        LOGI << "Max bounds: " << getMaxBounds(m_PoseGetter);

        // Create output directory (if necessary)
        filesystem::create_directory(m_Config.getOutputPath());

#ifndef LOCAL_DISPLAY
        // If we should stream output, run server thread
        if(m_Config.shouldStreamOutput()) {
            Net::Server server;
            m_Connection = std::make_unique<Net::Connection>(server.waitForConnection());
            m_NetSink = std::make_unique<Video::NetSink>(*m_Connection, config.getUnwrapRes(), "unwrapped");
            m_Connection->runInBackground();
        }
#endif

        // Create unwrapper if needed
        if (m_Camera->needsUnwrapping()) {
            m_Unwrapper = std::make_unique<ImgProc::OpenCVUnwrap360>(m_Camera->createUnwrapper(config.getUnwrapRes()));
        } else {
            m_Camera->setOutputSize(config.getUnwrapRes());
        }
        m_Output.create(m_Camera->getOutputSize(), CV_8UC3);
        m_DifferenceImage.create(m_Camera->getOutputSize(), CV_8UC1);

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
            m_Joystick->update();

            // Exit if X is pressed
            if(m_Joystick->isPressed(HID::JButton::X)) {
                 // If we should use Vicon capture control
                if(m_Config.shouldUseViconCaptureControl()) {
                    // Stop capture
                    m_ViconCaptureControl.stopRecording(m_Config.getViconCaptureControlName());
                }
                return false;
            }

            if(state != State::Testing) {
                // Drive motors using joystick
                m_Motor.drive(*m_Joystick, m_Config.getJoystickDeadzone());
            }

            // Update display
            m_CarSprite.setPose(m_PoseGetter->getPose());
            m_Display.update(m_TrainPath, m_TestPath, m_CarSprite);
            if(!m_Display.isOpen()) {
                return false;
            }

            // Capture frame
            m_Camera->readFrameSync(m_Output);

            // Unwrap frame
            if (m_Unwrapper) {
                m_Unwrapper->unwrap(m_Output, m_Unwrapped);
            } else {
                m_Unwrapped = m_Output;
            }
            cv::waitKey(1);
        }

        if(state == State::WaitToTrain) {
            if(event == Event::Enter) {
                LOGI << "Press B to start training" ;
            }
            else if(event == Event::Update) {
                if(m_Joystick->isPressed(HID::JButton::B)) {
                    m_StateMachine.transition(State::Training);
                }
            }
        }
        else if(state == State::Training) {
            if(event == Event::Enter) {
                LOGI << "Starting training";

                // Clear linestrip
                m_TrainPath.clear();

                // Open settings file and write unwrapper settings to it
                cv::FileStorage settingsFile((m_Config.getOutputPath() / "training_settings.yaml").str(), cv::FileStorage::WRITE);
                if (m_Unwrapper) {
                    settingsFile << "unwrapper" << *m_Unwrapper;
                }

                // Close log file if it's already open
                if(m_LogFile.is_open()) {
                    m_LogFile.close();
                }

                m_LogFile.open((m_Config.getOutputPath() / "training.csv").str());
                BOB_ASSERT(m_LogFile.good());

                // Write header
                m_LogFile << "Time [s], Filename";

                // If Vicon tracking is available, write additional header
                if(m_Config.shouldLogPose()) {
                    m_LogFile << ", X, Y, Z, Rx, Ry, Rz";
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

                // If A is pressed
                if(m_Joystick->isPressed(HID::JButton::A) || (m_Config.shouldAutoTrain() && (currentTime - m_LastTrainTime) > m_Config.getTrainInterval())) {
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
                    logPose();
                }
                // Otherwise, if B is pressed, go to testing
                else if(m_Joystick->isPressed(HID::JButton::B)) {
                    m_StateMachine.transition(State::WaitToTest);
                }
            }
            m_TrainPath.append(m_PoseGetter->getPosition());
        }
        else if(state == State::WaitToTest) {
            if(event == Event::Enter) {
                LOGI << "Press B to start testing" ;
            }
            else if(event == Event::Update) {
                if(m_Joystick->isPressed(HID::JButton::B)) {
                    m_StateMachine.transition(State::Testing);
                }
            }
        }
        else if(state == State::Testing) {
            if(event == Event::Enter) {
                LOGI << "Testing: finding snapshot";
                m_TestPath.clear();

                // Open settings file and write unwrapper settings to it
                cv::FileStorage settingsFile((m_Config.getOutputPath() / "testing_settings.yaml").str().c_str(), cv::FileStorage::WRITE);
                if (m_Unwrapper) {
                    settingsFile << "unwrapper" << *m_Unwrapper;
                }

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
                if(m_Config.shouldLogPose()) {
                    m_LogFile << ", X, Y, Z, Rx, Ry, Rz";
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

                    logPose();

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
                    if(m_Config.shouldStreamOutput()) {
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

                            // Annotated difference image
#ifdef LOCAL_DISPLAY
                            cv::imshow("Image difference", m_DifferenceImage);
                            if ((cv::waitKeyEx(1) & OS::KeyMask) == OS::KeyCodes::Escape) {
                                return false;
                            }
#else
                            m_NetSink->sendFrame(m_DifferenceImage);
#endif
                        }
                        else {
                            LOGW << "WARNING: Can only stream output from a perfect memory";
                        }
                    }

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
            m_TestPath.append(m_PoseGetter->getPosition());
        }
        else {
            LOGE << "Invalid state";
            return false;
        }
        return true;
    }

    void logPose()
    {
        // If logging pose
        if(m_Config.shouldLogPose()) {
            // Get tracking data
            const Pose3<millimeter_t, degree_t> pose = m_PoseGetter->getPose();
            const auto &position = pose.position();
            const auto &attitude = pose.attitude();

            // Write to CSV
            m_LogFile << ", " << position[0].value()
                      << ", " << position[1].value()
                      << ", " << position[2].value()
                      << ", " << attitude[0].value()
                      << ", " << attitude[1].value()
                      << ", " << attitude[2].value();
        }
        m_LogFile << std::endl;
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Configuration
    const Config &m_Config;

    // State machine
    FSM<State> m_StateMachine;

    // Motor driver
    TANK_TYPE m_Motor;

    // For getting agent's position
    POSE_GETTER_TYPE m_PoseGetter;

    // Camera interface
    std::unique_ptr<Video::Input> m_Camera;

    // Display
    Viz::SFMLWorld m_Display;
    Viz::SFMLWorld::CarAgent m_CarSprite;
    Viz::SFMLWorld::LineStrip m_TrainPath, m_TestPath;

    // Joystick interface
    std::unique_ptr<HID::JoystickBase<HID::JAxis, HID::JButton>> m_Joystick;

    // OpenCV images used to store raw camera frame and unwrapped panorama
    cv::Mat m_Output;
    cv::Mat m_Unwrapped;
    cv::Mat m_DifferenceImage;

    // OpenCV-based panorama unwrapper
    std::unique_ptr<ImgProc::OpenCVUnwrap360> m_Unwrapper;

    // Image processor
    std::unique_ptr<ImageInput> m_ImageInput;

    // Perfect memory
    std::unique_ptr<MemoryBase> m_Memory;

    // Last time at which a motor command was issued or a snapshot was trained
    TimePoint m_LastMotorCommandTime;
    TimePoint m_LastTrainTime;

    // Time at which testing or training started
    TimePoint m_RecordingStartTime;

    Milliseconds m_TestDuration;

    // Index of test image to write
    size_t m_TestImageIndex;

    // Vicon capture control interface
    Vicon::CaptureControl m_ViconCaptureControl;

    // CSV file containing logging
    std::ofstream m_LogFile;

    // How many snapshots has memory been trained on
    size_t m_NumSnapshots;

#ifndef LOCAL_DISPLAY
    // Server for streaming etc
    std::unique_ptr<Net::Connection> m_Connection;

    // Sink for video to send over server
    std::unique_ptr<Video::NetSink> m_NetSink;
#endif
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
