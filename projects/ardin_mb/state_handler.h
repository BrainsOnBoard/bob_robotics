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
#include "video/opengl/opengl.h"

// Libantworld includes
#include "antworld/common.h"
#include "antworld/renderer.h"
#include "antworld/render_target.h"
#include "antworld/render_target_input.h"
#include "antworld/route_ardin.h"

// Ardin MB includes
#include "opencv_texture.h"
#include "vector_field.h"

// Forward declarations
namespace BoBRobotics
{
namespace Navigation
{
    class VisualNavigationBase;
}
namespace AntWorld
{
    class SnapshotProcessor;
}
}

class VisualNavigationUI;

//----------------------------------------------------------------------------
// Enumerations
//----------------------------------------------------------------------------
enum class State
{
    Invalid,
    Training,
    Testing,
    BuildingRIDF,
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
        KeyForward,
        KeyBackward,
        KeyUp,
        KeyDown,
        KeyReset,
        KeyMax
    };

    StateHandler(const std::string &worldFilename, const std::string &routeFilename, float jitterSD, bool quitAfterTrain, bool autoTest,
                 meter_t pathHeight, const std::vector<float> &minBound, const std::vector<float> &maxBound,
                 BoBRobotics::AntWorld::SnapshotProcessor &snapshotProcessor, BoBRobotics::Navigation::VisualNavigationBase &visualNavigation,
                 VisualNavigationUI &visualNavigationUI);

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

    //! Load a route
    void loadRoute(const std::string &filename);

    //! Update UI
    bool handleUI();

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
    BoBRobotics::AntWorld::RenderTarget m_RenderTargetTopDown;
    BoBRobotics::AntWorld::RenderTarget m_RenderTargetPanoramic;

    //! OpenGL video input used for reading image from framebuffer
    BoBRobotics::AntWorld::RenderTargetInput m_Input;

    //! Route handler - implements the various bits of route-regularizing weirdness from original paper
    BoBRobotics::AntWorld::RouteArdin m_Route;

    //! OpenCV texture wrapper used to render final snapshot
    OpenCVTexture m_FinalSnapshotTexture;

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

    //! Distribution of position jitter
    std::normal_distribution<float> m_PositionJitterDistributionCM;

    //! Distribution of angles to turn for random walk
    std::uniform_real_distribution<float> m_RandomWalkAngleDistribution;

    //! Should we quit after training? (useful for automated parameter sweeping)
    const bool m_QuitAfterTrain;

    //! Should we automatically test after training? (useful for automated benchmarking)
    const bool m_AutoTest;

    //! Snapshot processor - takes screen images and pre-processes
    BoBRobotics::AntWorld::SnapshotProcessor &m_SnapshotProcessor;

    //! Height of path
    const meter_t m_PathHeight;

    //! Model used for visual navigation
    BoBRobotics::Navigation::VisualNavigationBase &m_VisualNavigation;

    //! Object used to handle visual navigation model-specific UI
    VisualNavigationUI &m_VisualNavigationUI;

    unsigned int m_CurrentVectorFieldPoint;
    std::vector<std::pair<units::angle::degree_t, float>> m_VectorFieldNovelty;

    std::vector<float> m_RIDFNovelty;
};
