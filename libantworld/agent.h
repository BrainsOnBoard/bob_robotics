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
template<class LengthUnit = meter_t, class AngleUnit = radian_t>
class AntAgent
  : Positionable<LengthUnit>
  , Rotatable<AngleUnit>
  , Video::OpenGL
{
public:
    AntAgent(GLFWwindow *window, Renderer &renderer, GLsizei readWidth, GLsizei readHeight);
    Vector3<LengthUnit> &getPosition() override;
    Vector3<AngleUnit> &getAttitude() override;
    void setPosition(LengthUnit x, LengthUnit y, LengthUnit z) override;
    void setAttitude(AngleUnit yaw, AngleUnit pitch, AngleUnit roll) override;
    bool readFrame(cv::Mat &frame) override;

private:
    Vector3<AngleUnit> m_Attitude{ 0_rad, 0_rad, 0_rad };
    Vector3<LengthUnit> m_Position{ 0_m, 0_m, 0_m };
    Renderer &m_Renderer;
    GLFWwindow *m_Window;
}; // AntAgent
} // AntWorld
} // BoBRobotics

namespace BoBRobotics {
namespace AntWorld {
template<class LengthUnit, class AngleUnit>
AntAgent<LengthUnit, AngleUnit>::AntAgent(GLFWwindow *window, Renderer &renderer,
                                          GLsizei readWidth, GLsizei readHeight)
  : Video::OpenGL(0, 0, readWidth, readHeight)
  , m_Renderer(renderer)
  , m_Window(window)
{}

template<class LengthUnit, class AngleUnit>
Vector3<LengthUnit> &
AntAgent<LengthUnit, AngleUnit>::getPosition()
{
    return m_Position;
}

template<class LengthUnit, class AngleUnit>
Vector3<AngleUnit> &
AntAgent<LengthUnit, AngleUnit>::getAttitude()
{
    return m_Attitude;
}

template<class LengthUnit, class AngleUnit>
void
AntAgent<LengthUnit, AngleUnit>::setPosition(LengthUnit x, LengthUnit y, LengthUnit z)
{
    m_Position[0] = x;
    m_Position[1] = y;
    m_Position[2] = z;
}

template<class LengthUnit, class AngleUnit>
void
AntAgent<LengthUnit, AngleUnit>::setAttitude(AngleUnit yaw, AngleUnit pitch, AngleUnit roll)
{
    m_Attitude[0] = yaw;
    m_Attitude[1] = pitch;
    m_Attitude[2] = roll;
}

template<class LengthUnit, class AngleUnit>
bool
AntAgent<LengthUnit, AngleUnit>::readFrame(cv::Mat &frame)
{
    // Clear colour and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render first person
    m_Renderer.renderPanoramicView(m_Position[0], m_Position[1], m_Position[2],
                                   m_Attitude[0], m_Attitude[1], m_Attitude[2],
                                   0, 0, m_ReadWidth, m_ReadHeight);

    // Swap front and back buffers
    glfwSwapBuffers(m_Window);

    // Read frame
    return Video::OpenGL::readFrame(frame);
}
} // AntWorld
} // BoBRobotics