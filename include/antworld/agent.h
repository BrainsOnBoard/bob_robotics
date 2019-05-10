#pragma once

// BoB robotics includes
#include "common/pose.h"
#include "hid/joystick.h"
#include "robots/robot.h"
#include "video/opengl/opengl.h"
#include "renderer.h"

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// Standard C++ includes
#include <memory>
#include <mutex>

namespace BoBRobotics {
namespace AntWorld {
using namespace units::literals;

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::AntAgent
//----------------------------------------------------------------------------
//! A simple agent with a position and a panoramic view of the current AntWorld
class AntAgent
  : public Robots::Robot
  , public Video::OpenGL
{
    using meter_t = units::length::meter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using degree_t = units::angle::degree_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

public:
    // I chose these values to be (roughly) desert ant-like -- AD
    static constexpr meters_per_second_t DefaultVelocity = 0.03_mps;
    static constexpr radians_per_second_t DefaultTurnSpeed = 200_deg_per_s;

    AntAgent(GLFWwindow *window, Renderer &renderer,
             const cv::Size &renderSize,
             meters_per_second_t velocity = DefaultVelocity,
             radians_per_second_t turnSpeed = DefaultTurnSpeed);

    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f);

    template<typename LengthUnit = meter_t>
    Vector3<LengthUnit> getPosition() const
    {
        return convertUnitArray<LengthUnit>(m_Pose.position());
    }

    template<typename AngleUnit = degree_t>
    std::array<AngleUnit, 3> getAttitude() const
    {
        return convertUnitArray<AngleUnit>(m_Pose.attitude());
    }

    template<typename LengthUnit = meter_t, typename AngleUnit = degree_t>
    Pose3<LengthUnit, AngleUnit> getPose() const
    {
        return m_Pose;
    }

    GLFWwindow *getWindow() const;

    radians_per_second_t getMaximumTurnSpeed() const;

    void setPosition(meter_t x, meter_t y, meter_t z);

    void setAttitude(degree_t yaw, degree_t pitch, degree_t roll);

    void moveTo(const Pose3<meter_t, degree_t> &pose);

    template<class Func>
    bool moveToSync(const Pose3<meter_t, degree_t> &pose, Func)
    {
        moveTo(pose);
        return true;
    }

    template<class Func>
    bool moveTo(const Pose3<meter_t, degree_t> &pose, Func)
    {
        moveTo(pose);
        return true;
    }

    bool update();

    bool isOpen() const;

    virtual bool readFrame(cv::Mat &frame) override;

    virtual void stopMoving() override;

    virtual void moveForward(float speed) override;

    virtual void turnOnTheSpot(float clockwiseSpeed) override;

    void updatePose(const units::time::second_t elapsedTime);

    static std::unique_ptr<GLFWwindow, std::function<void(GLFWwindow *)>> initialiseWindow(const cv::Size &size);

private:
    Pose3<meter_t, degree_t> m_Pose;
    meters_per_second_t m_Velocity;
    radians_per_second_t m_TurnSpeed;
    Renderer &m_Renderer;
    GLFWwindow *m_Window;
    std::mutex m_PoseMutex;
    float m_JoystickX = 0.f, m_JoystickY = 0.f;

    enum class MoveMode
    {
        NotMoving,
        MovingForward,
        Turning
    } m_MoveMode = MoveMode::NotMoving;

    float m_SpeedProportion; //! From -1 to 1: indicates proportion of max forward/turning speed
}; // AntAgent
} // AntWorld
} // BoBRobotics
