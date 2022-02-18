#include "common.h"
#include "config.h"
#include "image_input.h"
#include "memory.h"

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/fsm.h"
#include "common/path.h"
#include "common/progress_bar.h"
#include "common/stopwatch.h"
#include "common/timer.h"
#include "common/bn055_imu.h"
#include "hid/joystick.h"
#include "hid/robot_control.h"
#include "imgproc/opencv_unwrap_360.h"
#include "imgproc/mask.h"
#include "navigation/image_database.h"
#include "net/server.h"
#include "robots/robot_type.h"
#ifdef USE_VICON
#include "vicon/capture_control.h"
#include "vicon/udp.h"
#endif // USE_VICON
#include "video/input.h"
#include "video/netsink.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/optional.hpp"
#include "third_party/path.h"

// Standard C++ includes
#include <chrono>
#include <future>
#include <limits>
#include <memory>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::math;
using namespace units::time;

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

//------------------------------------------------------------------------
// RobotFSM
//------------------------------------------------------------------------
class RobotFSM : FSM<State>::StateHandler
{
    using Milliseconds = std::chrono::duration<double, std::milli>;
    using ImageDatabase = Navigation::ImageDatabase;
    
public:
    RobotFSM(const Config &config, BackgroundExceptionCatcher &backgroundEx)
      : m_Config(config)
      , m_StateMachine(this, State::Invalid)
      , m_Camera(getPanoramicCamera(config))
      , m_Output(m_Camera->getOutputSize(), CV_8UC3)
      , m_DifferenceImage(config.getCroppedRect().size(), CV_8UC1)
      , m_ImageInput(createImageInput(config, *m_Camera))
      , m_Memory(createMemory(config, m_ImageInput->getOutputSize()))
      , m_TrainDatabase(m_Config.getOutputPath(), m_Config.shouldTrain())
      , m_NumSnapshots(0)
      , m_BackgroundEx(backgroundEx)
    {
        // If actually driving the robot, it needs to be initialised
        if (m_Config.shouldDriveRobot()) {
            m_Robot.emplace();
        } else {
            LOGW << "Robot driving is disabled in config file";
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

        // If we should use Vicon tracking
        if(m_Config.shouldUseViconTracking()) {
#ifdef USE_VICON
            // Connect to port specified in config
            m_ViconTracking.connect(m_Config.getViconTrackingPort());
#else
            throw std::runtime_error("You must rebuild with -DUSE_VICON=ON");
#endif
        }

        // If we should use Vicon capture control
        if(m_Config.shouldUseViconCaptureControl()) {
#ifdef USE_VICON
            // Connect to capture host system specified in config
            m_ViconCaptureControl.connect(m_Config.getViconCaptureControlHost(), m_Config.getViconCaptureControlPort(),
                                          m_Config.getViconCaptureControlPath());

            // Start capture
            m_ViconCaptureControl.startRecording(m_Config.getViconCaptureControlName());
#else
            throw std::runtime_error("You must rebuild with -DUSE_VICON=ON");
#endif
        }

        // If we should train
        if(m_Config.shouldTrain()) {
            // Start in training state
            m_StateMachine.transition(State::WaitToTrain);
        } else {
            // Train the algorithm on the stored images
            m_Memory->trainRoute(m_TrainDatabase, *m_ImageInput,
                                 m_Config.getSkipFrames(), &m_BackgroundEx);

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
#ifdef USE_VICON
                 // If we should use Vicon capture control
                if(m_Config.shouldUseViconCaptureControl()) {
                    // Stop capture
                    m_ViconCaptureControl.stopRecording(m_Config.getViconCaptureControlName());
                }
#endif
                LOGD << "X button pressed. Terminating...";
                return false;
            }

            // If we're in a suitable state, drive motors using joystick
            if(m_Robot && (state == State::WaitToTrain || state == State::Training || state == State::WaitToTest)) {
                HID::drive(*m_Robot, m_Joystick, m_Config.getJoystickDeadzone(), m_Config.getJoystickGain());
            }

            // Capture frame
            m_Camera->readFrameSync(m_Output);

            // Process image (unwrap, crop etc.)
            std::tie(m_Processed, m_Mask) = m_ImageInput->processSnapshot(m_Output);

            // Transmit over network if desired
            if (m_Config.shouldStreamOutput()) {
                m_LiveNetSink->sendFrame(m_Processed);
            }
        }

        if(state == State::WaitToTrain) {
            if (event == Event::Enter) {
                LOGI << "Press B to start training" ;
            } else if (event == Event::Update) {
                if(m_Joystick.isPressed(HID::JButton::B)) {
                    m_StateMachine.transition(State::Training);
                }
            }
        }
        else if(state == State::Training) {
            if(event == Event::Enter) {
                LOGI << "Starting training";
                m_NumSnapshots = 0;

                // If we should stream, send state update
                if(m_Config.shouldStreamOutput()) {
                    m_LiveConnection->getSocketWriter().send("SNAPSHOT_BOT_STATE TRAINING\n");
                }

                // For saving images + metadata
                makeRecorder(m_TrainDatabase);

                // Reset train time and test image
                m_RecordingStopwatch.start();
                m_TrainingStopwatch.start();
            }
            else if(event == Event::Update) {
                // If A is pressed
                if(m_Joystick.isPressed(HID::JButton::A) || (m_Config.shouldAutoTrain() && m_TrainingStopwatch.elapsed() > m_Config.getTrainInterval())) {
                    // Update last train time
                    m_TrainingStopwatch.start();

                    // Train memory
                    if ((m_NumSnapshots++ % m_Config.getSkipFrames()) == 0) {
                        LOGI << "\tTrained snapshot";
                        m_ProcessedSnapshots.emplace_back(m_Processed, m_Mask);
                    }

                    // If we should stream output, send snapshot
                    if(m_Config.shouldStreamOutput()) {
                        m_SnapshotNetSink->sendFrame(m_Processed);
                    }

                    const double elapsed = static_cast<millisecond_t>(m_RecordingStopwatch.elapsed()).value();
                    // If Vicon tracking is available
                    if(m_Config.shouldUseViconTracking()) {
#ifdef USE_VICON
                        // Get tracking data
                        const auto objectData = m_ViconTracking.getObjectData(m_Config.getViconTrackingObjectName());
                        const std::array<degree_t, 3> imuData = imu->getEulerAngles();
                        m_Recorder->record(objectData.getPose(),
                                           m_Output,
                                           elapsed,
                                           objectData.getFrameNumber(),
                                           imuData[0].value(),
                                           imuData[1].value(),
                                           imuData[2].value());
                        
#endif
                    } else {
                        m_Recorder->record(Pose3<millimeter_t, degree_t>::nan(),
                                           m_Output, elapsed, -1);
                    }
                }

                // If B is pressed, go to testing
                if(m_Joystick.isPressed(HID::JButton::B)) {
                    m_StateMachine.transition(State::WaitToTest);
                }
            }
            else if(event == Event::Exit) {
                if (m_Robot) {
                    m_Robot->stopMoving();
                }

                // Write metadata to disk
                m_Recorder.reset();

                // Train the algorithm with the cached processed snapshots
                {
                    ProgressBar trainProgBar{ "Training", m_ProcessedSnapshots.size() };
                    for (const auto &snapshot : m_ProcessedSnapshots) {
                        m_BackgroundEx.check();

                        m_Memory->train(snapshot.first, snapshot.second);
                        trainProgBar.increment();
                    }
                }

                // Recover memory
                m_ProcessedSnapshots.clear();
            }
        }
        else if(state == State::WaitToTest) {
            if(event == Event::Enter) {
                LOGI << "Press B to start testing" ;
            }
            else if(event == Event::Update) {
                if(m_Joystick.isPressed(HID::JButton::B)) {
                    // If we should stream, send state update
                    if(m_Config.shouldStreamOutput()) {
                        m_LiveConnection->getSocketWriter().send("SNAPSHOT_BOT_STATE TESTING\n");
                    }

                    // Create new image database in subfolder of training database
                    const auto testingPath = m_TrainDatabase.getPath() / "testing";
                    filesystem::create_directory(testingPath);
                    m_TestDatabase = std::make_unique<ImageDatabase>(Path::getNewPath(testingPath));

                    // Extra fields for test algorithms
                    std::vector<std::string> fieldNames = m_Memory->getCSVFieldNames();

                    // For writing metadata
                    makeRecorder(*m_TestDatabase, std::move(fieldNames));

                    // We only save images if this option is set
                    m_Recorder->setSaveImages(m_Config.shouldSaveTestingDiagnostic());

                    // Reset test time and test image
                    m_RecordingStopwatch.start();
                    m_TestImageIndex = 0;

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
                m_Memory->test(m_Processed, m_Mask);

                // Extra info about the navigation algo
                LOGD << "Heading: " << static_cast<degree_t>(m_Memory->getBestHeading()).value() << "°";
                LOGD << "Image difference: " << m_Memory->getLowestDifference();
                const auto *pm = dynamic_cast<PerfectMemory *>(m_Memory.get());
                LOGD_IF(pm) << "Best-matching snapshot: " << pm->getBestSnapshotIndex();

                ImageDatabase::Entry entry;
                entry.extraFields["Timestamp [ms]"] = std::to_string(static_cast<millisecond_t>(m_RecordingStopwatch.elapsed()).value());

                // Extra algorithm-specific info
                m_Memory->setCSVFieldValues(entry.extraFields);

                // If Vicon tracking is available
                if(m_Config.shouldUseViconTracking()) {
#ifdef USE_VICON
                    // Get tracking data
                    const auto objectData = m_ViconTracking.getObjectData(m_Config.getViconTrackingObjectName());
                    entry.pose = objectData.getPose();
                    entry.extraFields["Frame"] = std::to_string(objectData.getFrameNumber());
#endif
                } else {
                    entry.extraFields["Frame"] = "-1";
                }

                // Save data
                m_Recorder->record(m_Output, std::move(entry));

                // If we should stream output
                if(m_Config.shouldStreamOutput()) {
                    // Send out snapshot
                    m_SnapshotNetSink->sendFrame(m_Processed);

                    // Attempt to dynamic cast memory to a perfect memory
                    PerfectMemory *perfectMemory = dynamic_cast<PerfectMemory*>(m_Memory.get());
                    if(perfectMemory != nullptr) {
                        // Send best snapshot
                        m_BestSnapshotNetSink->sendFrame(perfectMemory->getBestSnapshot());
                    }
                    else {
                        LOGW << "Can only stream output from a perfect memory";
                    }
                }

                // If we should turn, set timer and transition to turning state
                const auto turnSpeed = m_Config.getTurnSpeed(m_Memory->getBestHeading());
                if(turnSpeed.first > 0.0f) {
                    m_DriveTime = turnSpeed.second;
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
                if (m_Robot) {
                    // If we're driving forward, do so
                    if(state == State::DrivingForward) {
                        m_Robot->moveForward(m_Config.getMoveSpeed());
                    }
                    // Otherwise start turning
                    else {
                        const auto turnSpeed = m_Config.getTurnSpeed(m_Memory->getBestHeading());
                        const float motorTurn = (m_Memory->getBestHeading() <  0.0_deg) ? turnSpeed.first : -turnSpeed.first;
                        m_Robot->turnOnTheSpot(motorTurn);
                    }
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
            else if(event == Event::Exit && m_Robot) {
                m_Robot->stopMoving();
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

    void makeRecorder(ImageDatabase &database, std::vector<std::string> fieldNames = {})
    {
        fieldNames.emplace_back("Timestamp [ms]");

        // Also log Vicon frame number, if present
        fieldNames.emplace_back("Frame");

        // Also log the imu data
        fieldNames.emplace_back("IMU yaw [degrees]");
        fieldNames.emplace_back("IMU pitch [degrees]");
        fieldNames.emplace_back("IMU roll [degrees]");
        
        // Record as video file or images according to user's preference
        if (m_Config.shouldRecordVideo()) {
            /*
             * If auto-training we know the framerate, otherwise just
             * arbitrarily choose 15 fps for the video file.
             */
            auto fps = 15_Hz;
            if (m_Config.shouldAutoTrain()) {
                const auto period = m_Config.getTrainInterval();
                if (period == Milliseconds(0)) {
                    fps = m_Camera->getFrameRate();
                } else {
                    fps = 1 / units::time::second_t{ period };
                    if (fps > m_Camera->getFrameRate()) {
                        throw std::runtime_error("Requested training rate is higher than camera's FPS");
                    }
                }
            }
            m_Recorder = database.createVideoRouteRecorder(m_Camera->getOutputSize(),
                                                           fps,
                                                           m_Config.getVideoFileExtension(),
                                                           m_Config.getVideoCodec(),
                                                           std::move(fieldNames));
        } else {
            m_Recorder = database.createRouteRecorder("jpg", std::move(fieldNames));
        }

        // Save additional metadata
        auto &metadata = m_Recorder->getMetadataWriter();
        metadata << "config" << m_Config;
        metadata << "camera" << *m_Camera;
        m_ImageInput->writeMetadata(metadata);
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
    cv::Mat m_Processed;
    cv::Mat m_DifferenceImage;

    // Mask for image matching
    ImgProc::Mask m_Mask;

    // Image processor
    std::unique_ptr<ImageInput> m_ImageInput;

    // Algorithm memory
    std::unique_ptr<MemoryBase> m_Memory;

    // Store for processed snapshots
    std::vector<std::pair<cv::Mat, ImgProc::Mask>> m_ProcessedSnapshots;

    // Motor driver
    std::experimental::optional<ROBOT_TYPE> m_Robot;

    // Last time at which a motor command was issued or a snapshot was trained
    Stopwatch m_MoveStopwatch;
    Stopwatch m_TrainingStopwatch;

    // Time at which testing or training started
    Stopwatch m_RecordingStopwatch;

    // Index of test image to write
    size_t m_TestImageIndex;

#ifdef USE_VICON
    // Vicon tracking interface
    Vicon::UDPClient<Vicon::ObjectData> m_ViconTracking;
    std::unique_ptr<BN055> imu;

    // Vicon capture control interface
    Vicon::CaptureControl m_ViconCaptureControl;
#endif
    
    
    // For training data
    ImageDatabase m_TrainDatabase;

    // For testing data
    std::unique_ptr<ImageDatabase> m_TestDatabase;

    // For recording training and testing data
    std::unique_ptr<ImageDatabase::RouteRecorder> m_Recorder;

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

    // For catching exceptions on background threads
    BackgroundExceptionCatcher &m_BackgroundEx;
};
}   // Anonymous namespace

int bobMain(int argc, char *argv[])
{
    Config config;

    if (argc > 1 && strcmp(argv[1], "--dump-config") == 0) {
        // Dump default config values to disk without doing anything
        cv::FileStorage configFile("default_config.yaml", cv::FileStorage::WRITE);
        configFile << "config" << config;

        return EXIT_SUCCESS;
    }

    {
        // Load from file/database specified by arguments
        config.parseArgs(argc, argv);

        BackgroundExceptionCatcher backgroundEx;
        RobotFSM robot(config, backgroundEx);
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        backgroundEx.trapSignals();
        for (frame = 0; robot.update(); frame++) {
            // Check for background exceptions and re-throw
            backgroundEx.check();
        }

        const double msPerFrame = timer.get() / (double)frame;
        LOGI << "FPS:" << 1000.0 / msPerFrame;
    }

    return 0;
}
