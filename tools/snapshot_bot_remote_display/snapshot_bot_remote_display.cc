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
                m_OutputImage.setTo(CV_RGB(255, 0, 0));
            }
            else if(event == Event::Update) {

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
                m_OutputImage.setTo(CV_RGB(0, 255, 0));
            }
            else if(event == Event::Update) {
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
    // Members
    //------------------------------------------------------------------------
    // Configuration
    const Config &m_Config;

    // State machine
    FSM<State> m_StateMachine;

    cv::Mat m_OutputImage;

    cv::Mat m_TestSnapshot;
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

    cv::namedWindow("Output", cv::WINDOW_NORMAL);
    cv::resizeWindow("Output", config.getResolution().width, config.getResolution().height);

    // Create state machine
    DisplayFSM fsm(config);

    // Main loop
    while(true) {
        if(!fsm.update()) {
            break;
        }

    }
}
