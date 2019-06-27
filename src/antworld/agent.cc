// BoB robotics includes
#include "antworld/agent.h"
#include "common/macros.h"

// Standard C++ includes
#include <mutex>

using namespace units::literals;
using namespace units::velocity;
using namespace units::angular_velocity;

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

// Declarations
constexpr meters_per_second_t AntAgent::DefaultVelocity;
constexpr radians_per_second_t AntAgent::DefaultTurnSpeed;

AntAgent::AntAgent(GLFWwindow *window,
                   Renderer &renderer,
                   const cv::Size &renderSize,
                   meters_per_second_t velocity,
                   radians_per_second_t turnSpeed)
  : Video::OpenGL(renderSize)
  , m_Velocity(velocity)
  , m_TurnSpeed(turnSpeed)
  , m_Renderer(renderer)
  , m_Window(window)
{}

void
AntAgent::addJoystick(HID::Joystick &joystick, float deadZone)
{
    joystick.addHandler(
            [this, deadZone](HID::JAxis axis, float value) {
                switch (axis) {
                case HID::JAxis::LeftStickVertical:
                    m_JoystickY = value;
                    break;
                case HID::JAxis::LeftStickHorizontal:
                    m_JoystickX = value;
                    break;
                default:
                    return false;
                }

                if (fabs(m_JoystickY) > deadZone) {
                    moveForward(-m_JoystickY);
                } else if (fabs(m_JoystickX) > deadZone) {
                    turnOnTheSpot(m_JoystickX);
                } else {
                    stopMoving();
                }
                return true;
            });
}

GLFWwindow *
AntAgent::getWindow() const
{
    return m_Window;
}

radians_per_second_t
AntAgent::getMaximumTurnSpeed() const
{
    return m_TurnSpeed;
}

void
AntAgent::setPosition(meter_t x, meter_t y, meter_t z)
{
    BOB_ASSERT(m_MoveMode == MoveMode::NotMoving);
    m_Pose.position() = { x, y, z };
}

void
AntAgent::setAttitude(degree_t yaw, degree_t pitch, degree_t roll)
{
    BOB_ASSERT(m_MoveMode == MoveMode::NotMoving);
    m_Pose.attitude() = { yaw, pitch, roll };
}

void
AntAgent::moveTo(const Pose3<meter_t, degree_t> &pose)
{
    BOB_ASSERT(m_MoveMode == MoveMode::NotMoving);
    m_Pose = pose;
}

bool
AntAgent::update()
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
AntAgent::isOpen() const
{
    return !glfwWindowShouldClose(m_Window);
}

bool
AntAgent::readFrame(cv::Mat &frame)
{
    update();

    // Read frame
    return Video::OpenGL::readFrame(frame);
}

void
AntAgent::stopMoving()
{
    m_MoveMode = MoveMode::NotMoving;
}

void
AntAgent::moveForward(float speed)
{
    BOB_ASSERT(speed >= -1.f && speed <= 1.f);
    BOB_ASSERT(m_Pose.pitch() == 0_deg && m_Pose.roll() == 0_deg);

    m_MoveMode = MoveMode::MovingForward;
    m_SpeedProportion = speed;
}

void
AntAgent::turnOnTheSpot(float clockwiseSpeed)
{
    BOB_ASSERT(clockwiseSpeed >= -1.f && clockwiseSpeed <= 1.f);
    BOB_ASSERT(m_Pose.pitch() == 0_deg && m_Pose.roll() == 0_deg);

    m_MoveMode = MoveMode::Turning;
    m_SpeedProportion = clockwiseSpeed;
}

void
AntAgent::updatePose(const units::time::second_t elapsedTime)
{
    std::lock_guard<std::mutex> guard(m_PoseMutex);

    using namespace units::math;
    switch (m_MoveMode) {
    case MoveMode::MovingForward: {
        const meter_t dist = m_SpeedProportion * m_Velocity * elapsedTime;
        m_Pose.x() += dist * cos(m_Pose.yaw());
        m_Pose.y() += dist * sin(m_Pose.yaw());
    } break;
    case MoveMode::Turning:
        m_Pose.yaw() -= m_SpeedProportion * m_TurnSpeed * elapsedTime;
        while (m_Pose.yaw() > 360_deg) {
            m_Pose.yaw() -= 360_deg;
        }
        while (m_Pose.yaw() < 0_deg) {
            m_Pose.yaw() += 360_deg;
        }
        break;
    default:
        break;
    }
}

std::unique_ptr<GLFWwindow, std::function<void(GLFWwindow *)>>
AntAgent::initialiseWindow(const cv::Size &size)
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
