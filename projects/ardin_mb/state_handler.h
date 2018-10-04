#pragma once

// Standard C++ includes
#include <bitset>
#include <string>

// OpenCV includes
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "common/fsm.h"
#include "third_party/units.h"
#include "video/opengl.h"

// Libantworld includes
#include "libantworld/common.h"
#include "libantworld/renderer.h"
#include "libantworld/route_ardin.h"
#include "libantworld/snapshot_processor_ardin.h"

// Forward declarations
namespace BoBRobotics
{
namespace Navigation
{
    class VisualNavigationBase;
}
}

//----------------------------------------------------------------------------
// Enumerations
//----------------------------------------------------------------------------
enum class State
{
    Invalid,
    Training,
    Testing,
    RandomWalk,
    FreeMovement,
};

//----------------------------------------------------------------------------
// StateHandler
//----------------------------------------------------------------------------
class StateHandler : public BoBRobotics::FSM<State>::StateHandler
{
public:
    //------------------------------------------------------------------------
    // Enumerations
    //------------------------------------------------------------------------
    // Keys
    enum Key
    {
        KeyLeft,
        KeyRight,
        KeyUp,
        KeyDown,
        KeyReset,
        KeyTrainSnapshot,
        KeyTestSnapshot,
        KeySaveSnapshot,
        KeyMax
    };

    StateHandler(const std::string &worldFilename, const std::string &routeFilename, BoBRobotics::Navigation::VisualNavigationBase &visualNavigation);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void setKeyState(Key key, bool state) { m_KeyBits.set(key, state); }
    bool update(){ return m_StateMachine.update(); }

private:
    //------------------------------------------------------------------------
    // Typedefines
    //------------------------------------------------------------------------
    // Bitset used for passing which keys have been pressed between key callback and render loop
    typedef std::bitset<KeyMax> KeyBitset;

    //------------------------------------------------------------------------
    // FSM::StateHandler virtuals
    //------------------------------------------------------------------------
    virtual bool handleEvent(State state, Event event) override;

    // Private helpers
    void resetAntPosition();

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    BoBRobotics::FSM<State> m_StateMachine;
    KeyBitset m_KeyBits;

    // Host OpenCV array to hold pixels read from screen
    cv::Mat m_Snapshot;

    BoBRobotics::AntWorld::Renderer m_Renderer;

    BoBRobotics::Video::OpenGL m_Input;

    BoBRobotics::AntWorld::RouteArdin m_Route;

    //MBMemory m_Memory;

    BoBRobotics::AntWorld::SnapshotProcessorArdin m_SnapshotProcessor;

    //VectorField m_VectorField;

    // Position and angle of ant
    units::length::meter_t m_AntX;
    units::length::meter_t m_AntY;
    units::angle::degree_t m_AntHeading;

    // Position along training route
    size_t m_TrainPoint;

    size_t m_MaxTestPoint;

    unsigned int m_TestingScan;

    unsigned int m_NumTestErrors;
    unsigned int m_NumTestSteps;

    units::angle::degree_t m_BestTestHeading;
    float m_LowestTestDifference;


    // Model used for visual navigation
    BoBRobotics::Navigation::VisualNavigationBase &m_VisualNavigation;

    /*unsigned int m_CurrentVectorFieldPoint;
    unsigned int m_CurrentAngleFrames;
    std::vector<float> m_VectorFieldNovelty;*/
};