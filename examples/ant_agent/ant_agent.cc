// BoB robotics includes
#include "hid/joystick.h"
#include "libantworld/agent.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>
#include <tuple>

using namespace BoBRobotics;
using namespace std::literals;
using namespace units::length;
using namespace units::angle;

// Anonymous namespace
namespace {
void
handleGLFWError(int errorNumber, const char *message)
{
    std::cerr << "GLFW error number:" << errorNumber << ", message:" << message << std::endl;
}

void
handleGLError(GLenum, GLenum, GLuint, GLenum, GLsizei, const GLchar *message, const void *)
{
    throw std::runtime_error(message);
}
}

int
main()
{
    const int RenderWidth = 720;
    const int RenderHeight = 150;
    const meter_t AntHeight = 1_cm;

    HID::Joystick joystick;

    // Set GLFW error callback
    glfwSetErrorCallback(handleGLFWError);

    // Initialize the library
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return EXIT_FAILURE;
    }

    // Prevent window being resized
    glfwWindowHint(GLFW_RESIZABLE, false);

    // Create a windowed mode window and its OpenGL context
    GLFWwindow *window = glfwCreateWindow(RenderWidth, RenderHeight, "Ant world", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        std::cerr << "Failed to create window" << std::endl;
        return EXIT_FAILURE;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return EXIT_FAILURE;
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

    // Create renderer
    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    auto &world = renderer.getWorld();
    world.load("../../libantworld/world5000_gray.bin",
               { 0.0f, 1.0f, 0.0f },
               { 0.898f, 0.718f, 0.353f });
    const auto minBound = world.getMinBound();
    const auto maxBound = world.getMaxBound();

    // Create agent and put in the centre of the world
    AntWorld::AntAgent agent(window, renderer, RenderWidth, RenderHeight);
    agent.setPosition((maxBound[0] - minBound[0]) / 2, (maxBound[1] - minBound[1]) / 2, AntHeight);

    // Control the agent with a joystick
    agent.addJoystick(joystick);

    std::cout << "Press the B button to quit" << std::endl;
    std::tuple<meter_t, meter_t, degree_t> lastPose;
    while (!glfwWindowShouldClose(window) && !joystick.isDown(HID::JButton::B)) {
        joystick.update();

        const auto position = agent.getPosition<>();
        const Vector3<degree_t> attitude = agent.getAttitude<>();
        auto pose = std::make_tuple(position[0], position[1], attitude[0]);
        if (pose != lastPose) {
            std::cout << "Pose: " << position[0] << ", " << position[1] << ", " << attitude[0] << std::endl;
            lastPose = pose;
        }

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render first person
        renderer.renderPanoramicView(position[0], position[1], position[2],
                                     attitude[0], attitude[1], attitude[2],
                                     0, 0, RenderWidth, RenderHeight);

        // Swap front and back buffers
        glfwSwapBuffers(window);
    }
}
