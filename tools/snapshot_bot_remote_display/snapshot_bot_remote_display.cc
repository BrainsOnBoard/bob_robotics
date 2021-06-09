// Snapshot bot display includes
#include "config.h"

// BoB robotics includes
#include "common/fsm.h"
#include "navigation/differencers.h"
#include "navigation/insilico_rotater.h"
#include "os/keycodes.h"
#include "net/client.h"
#include "video/netsource.h"

// Third-party includes
#include "plog/Log.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <numeric>

using namespace BoBRobotics;

//------------------------------------------------------------------------
// Anonymous namespace
//------------------------------------------------------------------------
namespace
{
enum class State
{
    Invalid,
    Training,
    Testing,
};

//------------------------------------------------------------------------
// DisplayFSM
//------------------------------------------------------------------------
class DisplayFSM : public FSM<State>::StateHandler
{
public:
    DisplayFSM(const char *robotHost, const Config &config)
    :   m_Config(config), m_StateMachine(this, State::Invalid), m_LiveClient(robotHost, config.getLiveImagePort()),
        m_SnapshotClient(robotHost, config.getSnapshotPort()), m_BestSnapshotClient(robotHost, config.getTestingBestSnapshotPort()),
        m_LiveNetSource(m_LiveClient), m_SnapshotNetSource(m_SnapshotClient), m_BestSnapshotNetSource(m_BestSnapshotClient),
        m_OutputImage(config.getResolution(), CV_8UC3), m_SnapshotBotState(State::Invalid)
    {
        // Start background threads to read network data
        m_LiveClient.runInBackground();
        m_SnapshotClient.runInBackground();
        m_BestSnapshotClient.runInBackground();

        // Also attach command handlers to live client connection
        m_LiveClient.setCommandHandler("SNAPSHOT_BOT_STATE",
                                       [this](Net::Connection &connection, const Net::Command &command)
                                       {
                                           handleSnapshotBotState(connection, command);
                                       });

        // Load backgrounds
        if(!m_Config.getTrainingBackgroundFilename().empty()) {
            m_TrainingBackground = cv::imread(m_Config.getTrainingBackgroundFilename());
            BOB_ASSERT(m_TrainingBackground.size() == m_Config.getResolution());
            BOB_ASSERT(m_TrainingBackground.type() == CV_8UC3);
        }
        if(!m_Config.getTestingBackgroundFilename().empty()) {
            m_TestingBackground = cv::imread(m_Config.getTestingBackgroundFilename());
            BOB_ASSERT(m_TestingBackground.size() == m_Config.getResolution());
            BOB_ASSERT(m_TestingBackground.type() == CV_8UC3);
        }

        // Start in training state
        m_StateMachine.transition(State::Training);
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
    virtual bool handleEvent(State state, Event event) override
    {
        // If there is a state transition to apply, do so
        const State snapshotBotState = m_SnapshotBotState.exchange(State::Invalid);
        if(snapshotBotState != State::Invalid && state != snapshotBotState) {
            m_StateMachine.transition(snapshotBotState);
        }

        if(state == State::Training) {
            if(event == Event::Enter) {
                // Copy training background over output image
                if(m_TrainingBackground.empty()) {
                    m_OutputImage.setTo(cv::Scalar(255, 255, 255));
                }
                else {
                    m_TrainingBackground.copyTo(m_OutputImage);
                }
            }
            else if(event == Event::Update) {
                // Update live snapshot
                if(m_LiveNetSource.readFrame(m_ReceiveBuffer)) {
                    resizeAndProcessImageIntoOutputROI(m_Config.getTrainingLiveRect(), m_ReceiveBuffer);
                }

                // Update live snapshot
                if(m_SnapshotNetSource.readFrame(m_ReceiveBuffer)) {
                    // Convert received image into suitable format for displaying
                    processImage(m_ReceiveBuffer);

                    // Add image to receive buffer
                    m_TrainingSnapshots.emplace_back();
                    m_ReceiveBuffer.copyTo(m_TrainingSnapshots.back());

                    const int numTrainingSnapshots = m_TrainingSnapshots.size();
                    const int numRectangles = m_Config.getTrainingSnapshotRects().size();

                    const int numSnapshotsToDisplay = std::min(numTrainingSnapshots, numRectangles);
                    const int startDisplaySnapshot = std::max(0, numTrainingSnapshots - numRectangles);

                    // Loop through available rectangles
                    for(int i = 0; i < numSnapshotsToDisplay; i++) {
                        resizeImageIntoOutputROI(m_Config.getTrainingSnapshotRects()[i],
                                                 m_TrainingSnapshots[startDisplaySnapshot + i]);
                    }
                }

                // Show output image
                cv::imshow("Output", m_OutputImage);

                // Read input and update
                const int key = cv::waitKey(20);
                if((key & OS::KeyMask) == OS::KeyCodes::Escape) {
                    return false;
                }
            }
        }
        else if(state == State::Testing) {
            if(event == Event::Enter) {
                // Clear flags
                m_BestSnapshotReceived = false;
                m_SnapshotReceived = false;

                // Copy testing background over output image
                if(m_TestingBackground.empty()) {
                    m_OutputImage.setTo(cv::Scalar(255, 255, 255));
                }
                else {
                    m_TestingBackground.copyTo(m_OutputImage);
                }
            }
            else if(event == Event::Update) {
                // Update live snapshot
                if(m_LiveNetSource.readFrame(m_ReceiveBuffer)) {
                    resizeAndProcessImageIntoOutputROI(m_Config.getTestingLiveRect(), m_ReceiveBuffer);
                }

                // Update best snapshot
                if(m_BestSnapshotNetSource.readFrame(m_ReceiveBuffer)) {
                    if(m_BestSnapshotReceived) {
                        LOGW << "Best snapshot receiving out of sync";
                    }
                    else {
                        // Make a copy of received image
                        m_ReceiveBuffer.copyTo(m_BestSnapshot);

                        // Set flag
                        m_BestSnapshotReceived = true;
                    }
                }

                // Update snapshot that robot matched with best snapshot
                if(m_SnapshotNetSource.readFrame(m_ReceiveBuffer)) {
                    if(m_SnapshotReceived) {
                        LOGW << "Snapshot receiving out of sync";
                    }
                    else {
                        // Make a copy of received image
                        m_ReceiveBuffer.copyTo(m_Snapshot);

                        // Set flag
                        m_SnapshotReceived = true;
                    }
                }

                // If both images have been received
                if(m_BestSnapshotReceived && m_SnapshotReceived) {
                    BOB_ASSERT(m_Snapshot.type() == CV_8UC1);

                    const int imageWidth = m_Snapshot.size().width;
                    const int numScanColumns = (int)std::round(units::angle::turn_t(m_Config.getMaxSnapshotRotateAngle()).value() * (double)imageWidth);

                    // Ensure there is enough space in RIDF
                    m_RIDF.resize(numScanColumns * 2);

                    // Scan across columns on left of image
                    auto rotatorLeft = Navigation::InSilicoRotater::create(m_Snapshot.size(), {}, m_Snapshot,
                                                                           1, 0, numScanColumns);
                    rotatorLeft.rotate(
                            [numScanColumns, this]
                            (const cv::Mat &fr, const ImgProc::Mask &, size_t i)
                            {
                                // Store mean abs difference in RIDF
                                m_RIDF[numScanColumns - 1 - i] = this->m_Differencer(fr, m_BestSnapshot);
                            });


                    // Scan across columns on right of image
                    auto rotatorRight = Navigation::InSilicoRotater::create(m_Snapshot.size(), {}, m_Snapshot,
                                                                            1, imageWidth - numScanColumns, imageWidth);
                    rotatorRight.rotate(
                            [numScanColumns, this]
                            (const cv::Mat &fr, const ImgProc::Mask &, size_t i)
                            {
                                // Store mean abs difference in RIDF
                                m_RIDF[(2 * numScanColumns) - 1 - i] = this->m_Differencer(fr, m_BestSnapshot);
                            });

                    // Get range of
                    const auto minMaxRIDF = std::minmax_element(m_RIDF.cbegin(), m_RIDF.cend());
                    const float minRIDF = *minMaxRIDF.first;
                    const float maxRIDF = *minMaxRIDF.second;

                    // Get RIDF axis rect and calculate scale factors
                    const auto &ridfAxisRect = m_Config.getTestingRIDFAxisRect();
                    const float ridfXScale = (float)ridfAxisRect.width / (float)(numScanColumns * 2);
                    const float ridfYScale = (float)(ridfAxisRect.height - m_Config.getTestingRIDFLineThickness()) / (maxRIDF - minRIDF);

                    // Get axis ROI
                    cv::Mat roiRIDF(m_OutputImage, ridfAxisRect);

                    // Clear axis
                    roiRIDF.setTo(m_Config.getTestingRIDFBackgroundColour());

                    // Draw RIDF line graph
                    cv::Point lastPoint(0, ridfAxisRect.height - (int)std::round((m_RIDF[0] - minRIDF) * ridfYScale));
                    for(int i = 1; i < (2 * numScanColumns); i++) {
                        const cv::Point point(std::round(i * ridfXScale),
                                              ridfAxisRect.height - (int)std::round((m_RIDF[i] - minRIDF) * ridfYScale));
                        cv::line(roiRIDF, lastPoint, point, m_Config.getTestingRIDFLineColour(), m_Config.getTestingRIDFLineThickness(), cv::LINE_AA);
                        lastPoint = point;
                    }

                    // Get index of min RIDF element and hence calculate pixels to roll snapshot
                    // **YUCK** this information was clearly present deep within Navigation::InSilicoRotater
                    const int minRIDFIdx = (int)std::distance(m_RIDF.cbegin(), std::min_element(m_RIDF.cbegin(), m_RIDF.cend()));
                    const int rollPixels = (minRIDFIdx < numScanColumns) ? (numScanColumns - 1 - minRIDFIdx) : (imageWidth + numScanColumns - 1 - minRIDFIdx);
                    BOB_ASSERT(rollPixels >= 0);
                    BOB_ASSERT(rollPixels < imageWidth);

                    // Ensure rotated snapshot size and type matches original
                    m_RotatedSnapshot.create(m_Snapshot.size(), m_Snapshot.type());

                    // Loop through rows
                    for (int y = 0; y < m_Snapshot.rows; y++) {
                        // Get pointer to start of row
                        const uint8_t *rowPtr = m_Snapshot.ptr(y);
                        uint8_t *rowPtrOut = m_RotatedSnapshot.ptr(y);

                        // Rotate row to left by pixels
                        std::rotate_copy(rowPtr, rowPtr + rollPixels, rowPtr + m_Snapshot.cols, rowPtrOut);
                    }

                    // Calculate difference image
                    cv::absdiff(m_BestSnapshot, m_RotatedSnapshot, m_RotatedSnapshot);
                    cv::applyColorMap(m_RotatedSnapshot, m_RotatedSnapshot, cv::COLORMAP_HOT);
                    resizeAndProcessImageIntoOutputROI(m_Config.getTestingDifferenceRect(), m_RotatedSnapshot);

                    // Show best snapshot
                    resizeAndProcessImageIntoOutputROI(m_Config.getTestingBestRect(), m_BestSnapshot);

                    // Clear flags for next match
                    m_BestSnapshotReceived = false;
                    m_SnapshotReceived = false;
                }

                // Show output image
                cv::imshow("Output", m_OutputImage);

                // Read input and update
                const int key = cv::waitKey(1);
                if(key == 27) {
                    return false;
                }
            }
        }
        else {
            throw std::runtime_error("Invalid state");
            return false;
        }

        return true;
    }

    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    void handleSnapshotBotState(Net::Connection &, const Net::Command &command)
    {
        if(command[1] == "TRAINING") {
            m_SnapshotBotState = State::Training;
        }
        else if(command[1] == "TESTING") {
            m_SnapshotBotState = State::Testing;
        }
        else {
            throw std::runtime_error("Unknown snapshot bot state '" + command[1] + "'");
        }
    }

    void processImage(cv::Mat &image)
    {
        if(image.type() == CV_8UC1) {
            cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        }
        else if(image.type() != CV_8UC3) {
            LOGE << "Unsupported image type received";
        }
    }

    void resizeAndProcessImageIntoOutputROI(const cv::Rect &roi, cv::Mat &mat)
    {
        processImage(mat);
        resizeImageIntoOutputROI(roi, mat);
    }

    void resizeImageIntoOutputROI(const cv::Rect &roi, const cv::Mat &mat)
    {
        cv::Mat roiMat(m_OutputImage, roi);
        cv::resize(mat, roiMat, roiMat.size(), 0.0, 0.0, m_Config.getSnapshotInterpolationMethod());
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Configuration
    const Config &m_Config;

    // State machine
    FSM<State> m_StateMachine;

    // Network clients for maintaining connections to servers running on robot
    Net::Client m_LiveClient;
    Net::Client m_SnapshotClient;
    Net::Client m_BestSnapshotClient;

    // Net sources for reading video from server
    Video::NetSource m_LiveNetSource;
    Video::NetSource m_SnapshotNetSource;
    Video::NetSource m_BestSnapshotNetSource;

    // Background images
    cv::Mat m_TrainingBackground;
    cv::Mat m_TestingBackground;

    // Image used for compositing output
    cv::Mat m_OutputImage;

    // Temporary image used for receiving image data
    cv::Mat m_ReceiveBuffer;

    // Flags indicating whether image data required for RIDF calculation have arrived
    bool m_BestSnapshotReceived;
    bool m_SnapshotReceived;

    // Copy of snapshots for RIDF calculation
    cv::Mat m_BestSnapshot;
    cv::Mat m_Snapshot;

    // Scratch images for RIDF calculation
    Navigation::AbsDiff::Internal<> m_Differencer;
    cv::Mat m_RotatedSnapshot;

    // Array of training snapshots
    std::vector<cv::Mat> m_TrainingSnapshots;

    // State transition requests received from snapshot bot
    std::atomic<State> m_SnapshotBotState;

    // RIDF calculated from images
    std::vector<float> m_RIDF;
};
}   // Anonymous namespace

int main(int argc, char *argv[])
{
    assert(argc > 1);
    const char *robotHost = argv[1];
    const char *configFilename = (argc > 2) ? argv[2] : "config.yaml";

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

    cv::namedWindow("Output", cv::WINDOW_NORMAL);
    cv::resizeWindow("Output", config.getResolution().width, config.getResolution().height);
    cv::setWindowProperty("Output", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    // Create state machine
    DisplayFSM fsm(robotHost, config);

    // Main loop
    while(true) {
        if(!fsm.update()) {
            break;
        }

    }
}
