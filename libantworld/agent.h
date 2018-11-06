#pragma once

// BoB robotics includes
#include "../common/pose.h"
#include "../video/opengl.h"
#include "../robots/robot.h"
#include "common.h"
#include "renderer.h"

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// Standard C++ includes
#include <chrono>

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
    using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;

public:
    static constexpr meters_per_second_t DefaultVelocity = 300_mm / 1_s;
    static constexpr degrees_per_second_t DefaultTurnSpeed = 100_deg_per_s;

    AntAgent(GLFWwindow *window, Renderer &renderer, GLsizei readWidth, GLsizei readHeight,
             meters_per_second_t velocity = DefaultVelocity, degrees_per_second_t turnSpeed = DefaultTurnSpeed)
      : Video::OpenGL(0, 0, readWidth, readHeight)
      , m_Velocity(velocity)
      , m_TurnSpeed(turnSpeed)
      , m_Renderer(renderer)
      , m_Window(window)
    {}

    template<class LengthUnit = meter_t>
    Vector3<LengthUnit> getPosition()
    {
        updatePose();
        return convertUnitArray<LengthUnit>(m_Position);
    }

    template<class AngleUnit = degree_t>
    Vector3<AngleUnit> getAttitude()
    {
        updatePose();
        return convertUnitArray<AngleUnit>(m_Attitude);
    }

    void setPosition(meter_t x, meter_t y, meter_t z)
    {
        BOB_ASSERT(m_MoveMode == MoveMode::NotMoving);

        m_Position[0] = x;
        m_Position[1] = y;
        m_Position[2] = z;
    }

    void setAttitude(degree_t yaw, degree_t pitch, degree_t roll)
    {
        BOB_ASSERT(m_MoveMode == MoveMode::NotMoving);

        m_Attitude[0] = yaw;
        m_Attitude[1] = pitch;
        m_Attitude[2] = roll;
    }

    virtual bool readFrame(cv::Mat &frame) override
    {
        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render first person
        const auto size = getOutputSize();
        m_Renderer.renderPanoramicView(m_Position[0], m_Position[1], m_Position[2],
                                       m_Attitude[0], m_Attitude[1], m_Attitude[2],
                                       0, 0, size.width, size.height);

        // Swap front and back buffers
        glfwSwapBuffers(m_Window);

        // Read frame
        return Video::OpenGL::readFrame(frame);
    }

    virtual void stopMoving() override
    {
        updatePose();
        m_MoveMode = MoveMode::NotMoving;
    }

    virtual void moveForward(float speed) override
    {
        BOB_ASSERT(m_Attitude[1] == 0_deg && m_Attitude[2] == 0_deg);

        updatePose();
        m_MoveMode = MoveMode::MovingForward;
        m_MoveSpeed = speed;
    }

    virtual void turnOnTheSpot(float clockwiseSpeed) override
    {
        BOB_ASSERT(m_Attitude[1] == 0_deg && m_Attitude[2] == 0_deg);

        updatePose();
        m_MoveMode = MoveMode::Turning;
        m_MoveSpeed = clockwiseSpeed;
    }

private:
    using TimeType = std::chrono::time_point<std::chrono::high_resolution_clock>;

    Vector3<degree_t> m_Attitude{{ 0_deg, 0_deg, 0_deg }};
    Vector3<meter_t> m_Position{{ 0_m, 0_m, 0_m }};
    meters_per_second_t m_Velocity;
    degrees_per_second_t m_TurnSpeed;
    Renderer &m_Renderer;
    GLFWwindow *m_Window;

    TimeType m_MoveStartTime;
    enum class MoveMode
    {
        NotMoving,
        MovingForward,
        Turning
    } m_MoveMode = MoveMode::NotMoving;
    float m_MoveSpeed;

    void updatePose()
    {
        const TimeType newTime = now();
        const units::time::second_t elapsed(newTime - m_MoveStartTime);
        m_MoveStartTime = newTime;

        using namespace units::math;
        switch (m_MoveMode) {
        case MoveMode::MovingForward:
            {
                const meter_t dist = m_MoveSpeed * m_Velocity * elapsed;
                m_Position[0] += dist * cos(m_Attitude[0]);
                m_Position[1] += dist * sin(m_Attitude[0]);
            }
            break;
        case MoveMode::Turning:
            m_Attitude[0] -= m_MoveSpeed * m_TurnSpeed * elapsed;
            break;
        default:
            break;
        }
    }

    static TimeType now()
    {
        return std::chrono::high_resolution_clock::now();
    }
}; // AntAgent
} // AntWorld
} // BoBRobotics
