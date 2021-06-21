// Standard C++ includes
#include <chrono>
#include <fstream>
#include <future>
#include <limits>
#include <memory>

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/fsm.h"
#include "common/stopwatch.h"
#include "common/timer.h"
#include "hid/joystick.h"
#include "imgproc/opencv_unwrap_360.h"
#include "imgproc/mask.h"
#include "net/server.h"
#include "plog/Log.h"
#include "robots/robot_type.h"
#include "vicon/capture_control.h"
#include "vicon/udp.h"
#include "video/netsink.h"
#include "video/panoramic.h"
#ifdef USE_ODK2
#include "video/odk2/odk2.h"
#endif

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
    DrivingForward,
    Turning,
    PausedDrivingForward,
    PausedTurning,
};

// Bounds used for extracting masks from ODK2 images
const cv::Scalar odk2MaskLowerBound(1, 1, 1);
const cv::Scalar odk2MaskUpperBound(255, 255, 255);

//------------------------------------------------------------------------
// RobotFSM
//------------------------------------------------------------------------
class RobotFSM : FSM<State>::StateHandler
{
    using Seconds = std::chrono::duration<double, std::ratio<1>>;
    using Milliseconds = std::chrono::duration<double, std::milli>;

public:
    RobotFSM(const Config &config)
    :   m_Config(config), m_StateMachine(this, State::Invalid), m_Camera(getPanoramicCamera(config)),
        m_Output(m_Camera->getOutputSize(), CV_8UC3), m_Unwrapped(config.getUnwrapRes(), CV_8UC3), m_Cropped(config.getCroppedRect().size(), CV_8UC3),
        m_DifferenceImage(config.getCroppedRect().size(), CV_8UC1), m_ImageInput(createImageInput(config)),
        m_Memory(createMemory(config, m_ImageInput->getOutputSize())), m_Robot(), m_NumSnapshots(0)
    {
        m_LogFile.exceptions(std::ios::badbit | std::ios::failbit);

        // Create output directory (if necessary)
        filesystem::create_directory(m_Config.getOutputPath());

        // If camera image will need unwrapping, create unwrapper
        if(m_Camera->needsUnwrapping()) {
            m_Unwrapper = m_Camera->createUnwrapper(config.getUnwrapRes());
        }

        // If we should stream output, run server thread
        if(m_Config.shouldStreamOutput()) {
            // Spawn async tasks to wait for connections
            auto liveAsync = std::async(std::launch::async,
                                        [this]()
                                        {
                                            Net::Server server(m_Config.getServerListenPort());
                                            m_LiveConnection = server.waitForConnection();
                                        });

            auto snapshotAsync = std::async(std::launch::async,
                                            [this]()
                                            {
                                                Net::Server server(m_Config.getSnapshotServerListenPort());
                                                m_SnapshotConnection = server.waitForConnection();
                                            });

            auto bestSnapshotAsync = std::async(std::launch::async,
                                                [this]()
                                                {
                                                    Net::Server server(m_Config.getBestSnapshotServerListenPort());
                                                    m_BestSnapshotConnection = server.waitForConnection();
                                                });

            // Wait for all connections to be established
            liveAsync.wait();
            snapshotAsync.wait();
            bestSnapshotAsync.wait();

            // Create netsinks
            m_LiveNetSink = std::make_unique<Video::NetSink>(*m_LiveConnection, config.getCroppedRect().size(), "live");
            m_SnapshotNetSink = std::make_unique<Video::NetSink>(*m_SnapshotConnection, config.getCroppedRect().size(), "snapshot");
            m_BestSnapshotNetSink = std::make_unique<Video::NetSink>(*m_BestSnapshotConnection, config.getCroppedRect().size(), "best_snapshot");

            // Start background threads for transmitting images
            m_LiveConnection->runInBackground();
            m_SnapshotConnection->runInBackground();
            m_BestSnapshotConnection->runInBackground();
        }

         // If a static mask image is specified, set it as the mask
        if(!m_Config.getMaskImageFilename().empty()) {
            BOB_ASSERT(!m_Config.shouldUseODK2());
            m_Mask.set(m_Config.getMaskImageFilename());
        }

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
            // **TODO** save ODK2 masks
            BOB_ASSERT(!m_Config.shouldUseODK2());

            // If we're not using InfoMax or pre-trained weights don't exist
            if(!m_Config.shouldUseInfoMax() || !(m_Config.getOutputPath() / ("weights" + config.getTestingSuffix() + ".bin")).exists()) {
                LOGI << "Training on stored snapshots";
                for(m_NumSnapshots = 0;;m_NumSnapshots++) {
                    const auto filename = getSnapshotPath(m_NumSnapshots);

                    // If file exists
                    if(filename.exists()) {
                        std::cout << "." << std::flush;

                        // Load snapshot
                        const cv::Mat snapshot = cv::imread(filename.str());

                        // If we're using ODK2, extract mask from iamge
                        if(m_Config.shouldUseODK2()) {
                            m_Mask.set(m_Cropped, odk2MaskLowerBound, odk2MaskUpperBound);
                        }

                        // Process snapshot
                        const cv::Mat &processedSnapshot = m_ImageInput->processSnapshot(snapshot);

                        // Train model
                        m_Memory->train(processedSnapshot, m_Mask);
                    }
                    // Otherwise, stop searching
                    else {
                        break;
                    }
                }
                LOGI << "Loaded " << m_NumSnapshots << " snapshots";

                // If we are using InfoMax save the weights now
                if(m_Config.shouldUseInfoMax()) {
                    InfoMax *infoMax = dynamic_cast<InfoMax*>(m_Memory.get());
                    infoMax->saveWeights(m_Config.getOutputPath() / "weights.bin");
                }
            }

            // Start directly in testing state
            m_StateMachine.transition(State::WaitToTest);
        }
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

    std::unique_ptr<Video::Input> getPanoramicCamera(const Config &config)
    {
        if(config.shouldUseODK2()) {
#ifdef USE_ODK2
            return std::make_unique<Video::ODK2>();
#else
            throw std::runtime_error("Snapshot bot not compiled with ODK2 support - please re-run cmake with -DUSE_ODK2=1");
#endif
        }
        else {
            return Video::getPanoramicCamera(cv::CAP_V4L);
        }
    }

    //------------------------------------------------------------------------
    // FSM::StateHandler virtuals
    //------------------------------------------------------------------------
    bool handleEvent(State state, Event event) override
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

            // If we're in a suitable state, drive motors using joystick
            if(state == State::WaitToTrain || state == State::Training || state == State::WaitToTest) {
                m_Robot.drive(m_Joystick, m_Config.getJoystickDeadzone());
            }

            // Capture frame
            if(!m_Camera->readFrame(m_Output)) {
                return false;
            }

            // If our camera image needs unwrapping
            if(m_Camera->needsUnwrapping()) {
                // Unwrap the image
                m_Unwrapper.unwrap(m_Output, m_Unwrapped);

                // Crop unwrapped frame
                m_Cropped = cv::Mat(m_Unwrapped, m_Config.getCroppedRect());
            }
            // Otherwise, crop camera image directly
            else {
                m_Cropped = cv::Mat(m_Output, m_Config.getCroppedRect());
            }

            // If we're using the ODK2, generate mask from all black pixels in cropped image
            if(m_Config.shouldUseODK2()) {
                m_Mask.set(m_Cropped, odk2MaskLowerBound, odk2MaskUpperBound);
            }

            // Pump OpenCV event queue
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

                // If we should stream, send state update
                if(m_Config.shouldStreamOutput()) {
                    m_LiveConnection->getSocketWriter().send("SNAPSHOT_BOT_STATE TRAINING\n");
                }

                // Close log file if it's already open
                if(m_LogFile.is_open()) {
                    m_LogFile.close();
                }

                m_LogFile.open((m_Config.getOutputPath() / "training.csv").str());

                // Write header
                m_LogFile << "Time [s], Filename";

                // If Vicon tracking is available, write additional header
                if(m_Config.shouldUseViconTracking()) {
                    m_LogFile << ", Frame, X, Y, Z, Rx, Ry, Rz";
                }
                m_LogFile << std::endl;

                // Reset train time and test image
                m_RecordingStopwatch.start();
                m_TrainingStopwatch.start();

                // Delete old snapshots
                const std::string snapshotWildcard = (m_Config.getOutputPath() / "snapshot_*.png").str();
                system(("rm -f " + snapshotWildcard).c_str());
            }
            else if(event == Event::Update) {
                // If A is pressed
                if(m_Joystick.isPressed(HID::JButton::A) || (m_Config.shouldAutoTrain() && m_TrainingStopwatch.elapsed() > m_Config.getTrainInterval())) {
                    // Update last train time
                    m_TrainingStopwatch.start();

                    // Train memory
                    LOGI << "\tTrained snapshot";
                    const auto &processedSnapshot = m_ImageInput->processSnapshot(m_Cropped);
                    m_Memory->train(processedSnapshot, m_Config.shouldUseODK2() ? m_Mask.clone() : m_Mask);

                    // Write raw snapshot to disk
                    const std::string filename = getSnapshotPath(m_NumSnapshots++).str();
                    cv::imwrite(filename, m_Cropped);

                    // Write time
                    m_LogFile << ((Seconds)m_RecordingStopwatch.elapsed()).count() << ", " << filename;

                    // If we should stream output, send snapshot
                    if(m_Config.shouldStreamOutput()) {
                        m_SnapshotNetSink->sendFrame(processedSnapshot);
                    }

                    // If Vicon tracking is available
                    if(m_Config.shouldUseViconTracking()) {
                        // Get tracking data
                        const auto objectData = m_ViconTracking.getObjectData(m_Config.getViconTrackingObjectName());
                        const Pose3<millimeter_t, degree_t> pose = objectData.getPose();
                        const auto &position = pose.position();
                        const auto &attitude = pose.attitude();

                        // Write to CSV
                        m_LogFile << ", " << objectData.getFrameNumber() << ", ";
                        m_LogFile << position[0].value() << ", " << position[1].value() << ", " << position[2].value() << ", ";
                        m_LogFile << attitude[0].value() << ", " << attitude[1].value() << ", " << attitude[2].value();
                    }
                    m_LogFile << std::endl;
                }

                // If B is pressed, go to testing
                if(m_Joystick.isPressed(HID::JButton::B)) {
                    m_StateMachine.transition(State::WaitToTest);
                }
            }
            else if(event == Event::Exit) {
                m_Robot.stopMoving();
            }
        }
        else if(state == State::WaitToTest) {
            if(event == Event::Enter) {
                LOGI << "Press B to start testing" ;
            }
            else if(event == Event::Update) {
                if(m_Joystick.isPressed(HID::JButton::B)) {
                    // Open settings file and write unwrapper settings to it
                    cv::FileStorage settingsFile((m_Config.getOutputPath() / "testing_settings.yaml").str(), cv::FileStorage::WRITE);
                    settingsFile << "unwrapper" << m_Unwrapper;

                    // If we should stream, send state update
                    if(m_Config.shouldStreamOutput()) {
                        m_LiveConnection->getSocketWriter().send("SNAPSHOT_BOT_STATE TESTING\n");
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
                    if(m_Config.shouldUseViconTracking()) {
                        m_LogFile << ", Frame number, X, Y, Z, Rx, Ry, Rz";
                    }

                    if(m_Config.shouldSaveTestingDiagnostic()) {
                        m_LogFile << ", Filename";
                    }
                    m_LogFile << std::endl;

                    // Reset test time and test image
                    m_RecordingStopwatch.start();
                    m_TestImageIndex = 0;

                    // Delete old testing images
                    const std::string testWildcard = (m_Config.getOutputPath() / ("test" +  m_Config.getTestingSuffix() + "_*.png")).str();
                    system(("rm -f " + testWildcard).c_str());

                    m_StateMachine.transition(State::Testing);
                }
            }
        }
        else if(state == State::Testing) {
            if(event == Event::Enter) {
                LOGI << "Testing: finding snapshot" ;
            }
            else if(event == Event::Update) {
                // Find matching snapshot
                const auto &processedSnapshot = m_ImageInput->processSnapshot(m_Cropped);
                m_Memory->test(processedSnapshot, m_Mask);

                // Write time
                m_LogFile << ((Seconds)m_RecordingStopwatch.elapsed()).count() << ", ";

                // Write memory-specific CSV logging
                m_Memory->writeCSVLine(m_LogFile);

                // If vicon tracking is available
                if(m_Config.shouldUseViconTracking()) {
                    // Get tracking data
                    const auto objectData = m_ViconTracking.getObjectData();
                    const Pose3<millimeter_t, degree_t> pose = objectData.getPose();
                    const auto &position = pose.position();
                    const auto &attitude = pose.attitude();

                    // Write extra logging data
                    m_LogFile << ", " << objectData.getFrameNumber() << ", ";
                    m_LogFile << position[0].value() << ", " << position[1].value() << ", " << position[2].value() << ", ";
                    m_LogFile << attitude[0].value() << ", " << attitude[1].value() << ", " << attitude[2].value();
                }

                // If we should save diagnostics when testing
                if(m_Config.shouldSaveTestingDiagnostic()) {
                    const std::string filename = "test" + m_Config.getTestingSuffix() + "_" + std::to_string(m_TestImageIndex++) + ".png";
                    m_LogFile << ", " << filename;
                    // Build path to test image and save
                    const auto testImagePath = m_Config.getOutputPath() / filename;
                    cv::imwrite(testImagePath.str(), m_Cropped);
                }

                m_LogFile << std::endl;

                // If we should stream output
                if(m_Config.shouldStreamOutput()) {
                    // Send out snapshot
                    m_SnapshotNetSink->sendFrame(processedSnapshot);

                    // Attempt to dynamic cast memory to a perfect memory
                    PerfectMemory *perfectMemory = dynamic_cast<PerfectMemory*>(m_Memory.get());
                    if(perfectMemory != nullptr) {
                        // Send best snapshot
                        m_BestSnapshotNetSink->sendFrame(perfectMemory->getBestSnapshot());
                    }
                    else {
                        LOGW << "WARNING: Can only stream output from a perfect memory";
                    }
                }

                // If we should turn, set timer and transition to turning state
                if(m_Config.getTurnSpeed(m_Memory->getBestHeading()) > 0.0f) {
                    m_DriveTime = m_Config.getMotorTurnCommandInterval();
                    m_StateMachine.transition(State::Turning);
                }
                // Otherwise, set timer and transition to driving forward state
                else {
                    m_DriveTime = m_Config.getMotorCommandInterval();
                    m_StateMachine.transition(State::DrivingForward);
                }


            }
        }
        else if(state == State::DrivingForward || state == State::Turning) {
            if(event == Event::Enter) {
                // If we're driving forward, do so
                if(state == State::DrivingForward) {
                    m_Robot.moveForward(m_Config.getMoveSpeed());
                }
                // Otherwise start turning
                else {
                    const float turnSpeed = m_Config.getTurnSpeed(m_Memory->getBestHeading());
                    const float motorTurn = (m_Memory->getBestHeading() <  0.0_deg) ? turnSpeed : -turnSpeed;
                    m_Robot.turnOnTheSpot(motorTurn);
                }

                // Start timer
                m_MoveStopwatch.start();
            }
            else if(event == Event::Update) {
                // If A is pressed
                if(m_Joystick.isPressed(HID::JButton::A)) {
                    // Subtract time we've already moved for from drive time
                    m_DriveTime -= m_MoveStopwatch.elapsed();

                    // Transition to correct paused state
                    m_StateMachine.transition((state == State::DrivingForward) ? State::PausedDrivingForward : State::PausedTurning);
                }
                // Otherwise, if drive time has passed
                else if(m_MoveStopwatch.elapsed() > m_DriveTime) {
                    m_StateMachine.transition(State::Testing);
                }
            }
            else if(event == Event::Exit) {
                m_Robot.stopMoving();
            }
        }
        else if(state == State::PausedDrivingForward || state == State::PausedTurning) {
            if(event == Event::Update) {
                // If A is pressed, transition back to correct movement state
                if(m_Joystick.isPressed(HID::JButton::A)) {
                    m_StateMachine.transition((state == State::PausedDrivingForward) ? State::DrivingForward : State::Turning);
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
    cv::Mat m_Cropped;
    cv::Mat m_DifferenceImage;

    // OpenCV-based panorama unwrapper
    ImgProc::OpenCVUnwrap360 m_Unwrapper;

    // Mask for image matching
    ImgProc::Mask m_Mask;

    // Image processor
    std::unique_ptr<ImageInput> m_ImageInput;

    // Perfect memory
    std::unique_ptr<MemoryBase> m_Memory;

    // Motor driver
    Robots::ROBOT_TYPE m_Robot;

    // Last time at which a motor command was issued or a snapshot was trained
    Stopwatch m_MoveStopwatch;
    Stopwatch m_TrainingStopwatch;

    // Time at which testing or training started
    Stopwatch m_RecordingStopwatch;

    // Index of test image to write
    size_t m_TestImageIndex;

    // Vicon tracking interface
    Vicon::UDPClient<Vicon::ObjectData> m_ViconTracking;

    // Vicon capture control interface
    Vicon::CaptureControl m_ViconCaptureControl;

    // CSV file containing logging
    std::ofstream m_LogFile;

    Milliseconds m_DriveTime;

    // How many snapshots has memory been trained on
    size_t m_NumSnapshots;

    // Connections for streaming live images
    std::unique_ptr<Net::Connection> m_LiveConnection;
    std::unique_ptr<Net::Connection> m_SnapshotConnection;
    std::unique_ptr<Net::Connection> m_BestSnapshotConnection;

    // Sinks for video to send over server
    std::unique_ptr<Video::NetSink> m_LiveNetSink;
    std::unique_ptr<Video::NetSink> m_SnapshotNetSink;
    std::unique_ptr<Video::NetSink> m_BestSnapshotNetSink;
};
}   // Anonymous namespace

int bobMain(int argc, char *argv[])
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
    BackgroundExceptionCatcher backgroundEx;
    RobotFSM robot(config);

    {
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0; robot.update(); frame++) {
        }

        // Check for background exceptions and re-throw
        backgroundEx.check();

        const double msPerFrame = timer.get() / (double)frame;
        LOGI << "FPS:" << 1000.0 / msPerFrame;
    }

    return 0;
}
