// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "common/fsm.h"
#include "common/logging.h"

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
    DisplayFSM(const Config &config)
    :   m_Config(config), m_StateMachine(this, State::Invalid), m_OutputImage(config.getResolution(), CV_8UC3)
    {
        // **TEMP** load a snapshot
        m_TestSnapshot = cv::imread("snapshot_197.png");

        m_LiveSnapshot = m_TestSnapshot;
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
                m_OutputImage.setTo(CV_RGB(255, 255, 255));
            }
            else if(event == Event::Update) {
                // Update live snapshot
                //if(m_LiveSnapshotNetsink.readFrame(m_LiveSnapshot)) {
                    resizeImageIntoOutputROI(m_Config.getTestingLiveRect(), m_LiveSnapshot);
                //}

                // Show output image
                cv::imshow("Output", m_OutputImage);

                // Read input and update
                const int key = cv::waitKey(1);
                if(key == 27) {
                    return false;
                }
                else if(key == 's') {
                    m_TrainingSnapshots.push_back(m_TestSnapshot);
                    updateTrainingSnapshotDisplay();
                }
                else if(key == 't') {
                    m_StateMachine.transition(State::Testing);
                }
            }
        }
        else if(state == State::Testing) {
            if(event == Event::Enter) {
                // **TODO** load background
                m_OutputImage.setTo(CV_RGB(255, 255, 255));
            }
            else if(event == Event::Update) {
                // Update live snapshot
                //if(m_LiveSnapshotNetsink.readFrame(m_LiveSnapshot)) {
                    resizeImageIntoOutputROI(m_Config.getTestingLiveRect(), m_LiveSnapshot);
                //}

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
    void resizeImageIntoOutputROI(const cv::Rect &roi, const cv::Mat &mat)
    {
        cv::Mat roiMat(m_OutputImage, roi);
        cv::resize(mat, roiMat, roiMat.size(), 0.0, 0.0, m_Config.getSnapshotInterpolationMethod());

    }
    void updateTrainingSnapshotDisplay()
    {
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

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Configuration
    const Config &m_Config;

    // State machine
    FSM<State> m_StateMachine;

    // Image used for compositing output
    cv::Mat m_OutputImage;

    cv::Mat m_TestSnapshot;

    cv::Mat m_LiveSnapshot;
    std::vector<cv::Mat> m_TrainingSnapshots;
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

    cv::namedWindow("Output", cv::WINDOW_FULLSCREEN);
    cv::resizeWindow("Output", config.getResolution().width, config.getResolution().height);
    cv::setWindowProperty("Output", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    // Create state machine
    DisplayFSM fsm(config);

    // Main loop
    while(true) {
        if(!fsm.update()) {
            break;
        }

    }
}
