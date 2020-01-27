// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "common/fsm.h"
#include "common/logging.h"
#include "net/client.h"
#include "video/netsource.h"


// Snapshot bot display includes
#include "config.h"

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
        m_OutputImage(config.getResolution(), CV_8UC3)
    {
        // Start background threads to read network data
        m_LiveClient.runInBackground();
        m_SnapshotClient.runInBackground();
        m_BestSnapshotClient.runInBackground();

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
        if(state == State::Training) {
            if(event == Event::Enter) {
                // **TODO** load background
                m_OutputImage.setTo(cv::Scalar(255, 255, 255));

                cv::putText(m_OutputImage, "TRAINING", cv::Point(0, 20),
                            cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 0, 0));
            }
            else if(event == Event::Update) {
                // Update live snapshot
                if(m_LiveNetSource.readFrame(m_ReceiveBuffer)) {
                    processReceivedImage();

                    resizeImageIntoOutputROI(m_Config.getTestingLiveRect(), m_ReceiveBuffer);
                }

                // Update live snapshot
                if(m_SnapshotNetSource.readFrame(m_ReceiveBuffer)) {
                    processReceivedImage();

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
                const int key = cv::waitKey(1);
                if(key == 27) {
                    return false;
                }
                else if(key == 't') {
                    m_StateMachine.transition(State::Testing);
                }
            }
        }
        else if(state == State::Testing) {
            if(event == Event::Enter) {
                // **TODO** load background
                m_OutputImage.setTo(cv::Scalar(255, 255, 255));

                cv::putText(m_OutputImage, "TESTING", cv::Point(0, 20),
                            cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 0, 0));
            }
            else if(event == Event::Update) {
                // Update live snapshot
                if(m_LiveNetSource.readFrame(m_ReceiveBuffer)) {
                    processReceivedImage();
                    resizeImageIntoOutputROI(m_Config.getTestingLiveRect(), m_ReceiveBuffer);
                }

                if(m_BestSnapshotNetSource.readFrame(m_ReceiveBuffer)) {
                    processReceivedImage();
                    resizeImageIntoOutputROI(m_Config.getTestingBestRect(), m_ReceiveBuffer);
                }

                // Show output image
                cv::imshow("Output", m_OutputImage);

                // Read input and update
                const int key = cv::waitKey(1);
                if(key == 27) {
                    return false;
                }
                else if(key == 't') {
                    m_StateMachine.transition(State::Training);
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
    // Private methods
    //------------------------------------------------------------------------
    void processReceivedImage()
    {
        if(m_ReceiveBuffer.type() == CV_8UC1) {
            cv::cvtColor(m_ReceiveBuffer, m_ReceiveBuffer, cv::COLOR_GRAY2BGR);
        }
        else if(m_ReceiveBuffer.type() != CV_8UC3) {
            LOGE << "Unsupported image type received";
        }
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

    // Image used for compositing output
    cv::Mat m_OutputImage;

    // Temporary image used for receiving image data
    cv::Mat m_ReceiveBuffer;

    // Array of training snapshots
    std::vector<cv::Mat> m_TrainingSnapshots;
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

    cv::namedWindow("Output", cv::WINDOW_FULLSCREEN);
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
