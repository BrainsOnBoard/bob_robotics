#pragma once

// BoB robotics includes
#include "../common/pose.h"
#include "../video/opengl.h"
#include "common.h"
#include "renderer.h"

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

namespace BoBRobotics {
namespace AntWorld {
using namespace units::literals;

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::AntAgent
//----------------------------------------------------------------------------
//! A simple agent with a position and a panoramic view of the current AntWorld
class AntAgent
  : public Video::OpenGL
{
    using degree_t = units::angle::degree_t;
    using meter_t = units::length::meter_t;

public:
    AntAgent(GLFWwindow *window, Renderer &renderer, GLsizei readWidth, GLsizei readHeight)
      : Video::OpenGL(0, 0, readWidth, readHeight)
      , m_Renderer(renderer)
      , m_Window(window)
    {}

    template<class LengthUnit = meter_t>
    Vector3<LengthUnit> getPosition() const
    {
        return convertUnitArray<LengthUnit>(m_Position);
    }

    template<class AngleUnit = degree_t>
    Vector3<AngleUnit> getAttitude() const
    {
        return convertUnitArray<AngleUnit>(m_Attitude);
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

    static auto initialiseWindow(const cv::Size &size)
    {
        // Set GLFW error callback
        glfwSetErrorCallback(handleGLFWError);

        // Initialize the library
        if (!glfwInit()) {
            throw std::runtime_error("Failed to initialize GLFW");
        }

        // Prevent window being resized
        glfwWindowHint(GLFW_RESIZABLE, false);

        GLFWwindow *ptr = glfwCreateWindow(size.width, size.height, "Ant world", nullptr, nullptr);
        if (!ptr) {
            glfwTerminate();
            throw std::runtime_error("Failed to create window");
        }

        // Wrap in a unique_ptr so we can free it properly when we're done
        std::unique_ptr<GLFWwindow, std::function<void(GLFWwindow *)>> window(ptr, &glfwDestroyWindow);

        // Make the window's context current
        glfwMakeContextCurrent(window.get());

        // Initialize GLEW
        if (glewInit() != GLEW_OK) {
            throw std::runtime_error("Failed to initialize GLEW");
        }

        // Enable VSync
        glfwSwapInterval(1);

        glDebugMessageCallback(handleGLError, nullptr);

        // Set clear colour to match matlab and enable depth test
        glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glLineWidth(4.0);
        glPointSize(4.0);

        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);

        glEnable(GL_TEXTURE_2D);

        return window;
    }

private:
    Vector3<degree_t> m_Attitude{{ 0_deg, 0_deg, 0_deg }};
    Vector3<meter_t> m_Position{{ 0_m, 0_m, 0_m }};
    Renderer &m_Renderer;
    GLFWwindow *m_Window;

    static void handleGLFWError(int errorNumber, const char *message)
    {
        throw std::runtime_error("GLFW error number: " + std::to_string(errorNumber) + ", message:" + message);
    }

    static void handleGLError(GLenum, GLenum, GLuint, GLenum, GLsizei, const GLchar *message, const void *)
    {
        throw std::runtime_error(message);
    }

}; // AntAgent
} // AntWorld
} // BoBRobotics
