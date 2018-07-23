#pragma once

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// Libantworld includes
#include "common.h"
#include "renderer.h"

// BoB robotics includes
#include "../common/pose.h"
#include "../video/opengl.h"

using namespace BoBRobotics::Pose;

namespace BoBRobotics {
namespace AntWorld {
class AntAgent
  : Video::OpenGL
{
public:
    AntAgent(GLFWwindow *window, Renderer &renderer, GLsizei readWidth, GLsizei readHeight)
      : Video::OpenGL(0, 0, readWidth, readHeight)
      , m_Renderer(renderer)
      , m_Window(window)
    {}

    template<class LengthUnit = meter_t>
    Vector3<LengthUnit> getPosition() const
    {
        return makeUnitArray<LengthUnit>(m_Position);
    }

    template<class AngleUnit = degree_t>
    Vector3<AngleUnit> getAttitude() const
    {
        return makeUnitArray<AngleUnit>(m_Attitude);
    }

    void setPosition(meter_t x, meter_t y, meter_t z)
    {
        m_Position[0] = x;
        m_Position[1] = y;
        m_Position[2] = z;
    }

    void setAttitude(degree_t yaw, degree_t pitch, degree_t roll)
    {
        m_Attitude[0] = yaw;
        m_Attitude[1] = pitch;
        m_Attitude[2] = roll;
    }

    bool readFrame(cv::Mat &frame)
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

private:
    Vector3<degree_t> m_Attitude{ 0_deg, 0_deg, 0_deg };
    Vector3<meter_t> m_Position{ 0_m, 0_m, 0_m };
    Renderer &m_Renderer;
    GLFWwindow *m_Window;
}; // AntAgent
} // AntWorld
} // BoBRobotics