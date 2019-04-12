#pragma once

// Standard C++ includes
#include <bitset>
#include <random>
#include <string>

// OpenCV includes
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "common/fsm.h"
#include "common/pose.h"
#include "third_party/units.h"
#include "video/opengl.h"

// Libantworld includes
#include "antworld/common.h"
#include "antworld/renderer.h"
#include "antworld/route_ardin.h"
#include "antworld/snapshot_processor_ardin.h"

// Ardin MB includes
#include "vector_field.h"

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
    BuildingVectorField,
};

//----------------------------------------------------------------------------
// StateHandler
//----------------------------------------------------------------------------
class StateHandler : public BoBRobotics::FSM<State>::StateHandler
{
    using meter_t = units::length::meter_t;
    using degree_t = units::angle::degree_t;

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
        KeyRandomWalk,
        KeyBuildVectorField,
        KeyMax
    };

    StateHandler(const std::string &worldFilename, const std::string &routeFilename,
                 BoBRobotics::Navigation::VisualNavigationBase &visualNavigation);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void setKeyState(Key key, bool state) { m_KeyBits.set(key, state); }
    bool update(){ return m_StateMachine.update(); }

private:
    //------------------------------------------------------------------------
    // Typedefines
    //------------------------------------------------------------------------
    //! Bitset used for passing which keys have been pressed between key callback and render loop
    typedef std::bitset<KeyMax> KeyBitset;

    //------------------------------------------------------------------------
    // FSM::StateHandler virtuals
    //------------------------------------------------------------------------
    virtual bool handleEvent(State state, Event event) override;

    //------------------------------------------------------------------------
    // Private helpers
    //------------------------------------------------------------------------
    //! Move ant back to start of route/arbitrary location
    void resetAntPosition();

    //! Checks whether current position is still on route/at end etc
    bool checkAntPosition();

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    //! Finite state machine for ant world
    BoBRobotics::FSM<State> m_StateMachine;

    //! Bitset of current key states
    KeyBitset m_KeyBits;

    //! Host OpenCV array to hold pixels read from screen
    cv::Mat m_Snapshot;

    //! Renderer used for ant world
    BoBRobotics::AntWorld::Renderer m_Renderer;

    //! OpenGL video input used for reading image from framebuffer
    BoBRobotics::Video::OpenGL m_Input;

    //! Route handler - implements the various bits of route-regularizing weirdness from original paper
    BoBRobotics::AntWorld::RouteArdin m_Route;

    //! Snapshot processor - implements the strange resizing algorithm from original paper
    BoBRobotics::AntWorld::SnapshotProcessorArdin m_SnapshotProcessor;

    //! Class for handling rendering of vector field
    VectorField m_VectorField;

    //! Ant's current pose
    BoBRobotics::Pose2<meter_t, degree_t> m_Pose;

    //! When training, index of current snapshot
    size_t m_TrainPoint;

    //! The furthest point ant has reached along route
    size_t m_MaxTestPoint;

    //! Index of orientation when scanning world
    unsigned int m_TestingScan;

    //! The lowest novelty encountered when scanning world
    float m_LowestTestDifference;

    //! Angle where lowest novelty was encountered
    units::angle::degree_t m_BestTestHeading;

    //! Counters for number of errors made when testing (or random walking)
    unsigned int m_NumTestErrors;

    //! Counters for total number of steps made when testing (or random walking)
    unsigned int m_NumTestSteps;

    //! RNG used for random walk
    std::mt19937 m_RNG;

    //! Distribution of angles to turn for random walk
    std::uniform_real_distribution<float> m_RandomWalkAngleDistribution;

    //! Model used for visual navigation
    BoBRobotics::Navigation::VisualNavigationBase &m_VisualNavigation;

    unsigned int m_CurrentVectorFieldPoint;
    std::vector<std::pair<units::angle::degree_t, float>> m_VectorFieldNovelty;
};