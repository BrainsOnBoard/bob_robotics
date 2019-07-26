// BoB robotics includes
#include "antworld/camera.h"

void
handleGLFWError(int errorNumber, const char *message)
{
    throw std::runtime_error("GLFW error number: " + std::to_string(errorNumber) + ", message:" + message);
}

void
handleGLError(GLenum, GLenum, GLuint, GLenum, GLsizei, const GLchar *message, const void *)
{
    throw std::runtime_error(message);
}

namespace BoBRobotics {
namespace AntWorld {

Camera::Camera(GLFWwindow *window, Renderer &renderer, const cv::Size &renderSize)
  : Video::OpenGL(renderSize)
  , m_Window(window)
  , m_Renderer(renderer)
{}

GLFWwindow *
Camera::getWindow() const
{
    return m_Window;
}

void
Camera::setPose(const Pose3<meter_t, degree_t> &pose)
{
    m_Pose = pose;
}

void
Camera::setPosition(meter_t x, meter_t y, meter_t z)
{
    m_Pose.position() = { x, y, z };
}

void
Camera::setAttitude(degree_t yaw, degree_t pitch, degree_t roll)
{
    m_Pose.attitude() = { yaw, pitch, roll };
}

bool
Camera::readFrame(cv::Mat &frame)
{
    update();

    // Read frame
    return Video::OpenGL::readFrame(frame);
}

bool
Camera::update()
{
    // Render to m_Window
    glfwMakeContextCurrent(m_Window);

    // Clear colour and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render first person
    const auto size = getOutputSize();
    m_Renderer.renderPanoramicView(m_Pose.x(), m_Pose.y(), m_Pose.z(), m_Pose.yaw(), m_Pose.pitch(), m_Pose.roll(), 0, 0, size.width, size.height);

    // Swap front and back buffers
    glfwSwapBuffers(m_Window);

    return true;
}

bool
Camera::isOpen() const
{
    return !glfwWindowShouldClose(m_Window);
}

std::unique_ptr<GLFWwindow, std::function<void(GLFWwindow *)>>
Camera::initialiseWindow(const cv::Size &size)
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

} // AntWorld
} // BoBRobotics
