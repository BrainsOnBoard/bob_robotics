// Standard C++ includes
#include <iostream>

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// BoB robotics includes
#include "../hid/joystick.h"

// Libantworld includes
#include "../libantworld/common.h"
#include "../libantworld/renderer.h"

using namespace BoBRobotics;

// Anonymous namespace
namespace
{
void handleGLFWError(int errorNumber, const char *message)
{
    std::cerr << "GLFW error number:" << errorNumber << ", message:" << message << std::endl;
}

void handleGLError(GLenum source,
                   GLenum type,
                   GLuint id,
                   GLenum severity,
                   GLsizei length,
                   const GLchar *message,
                   const void *userParam)
{
    throw std::runtime_error(message);
}
}

int main()
{
    const float turnSpeed = 200.0f;
    const float moveSpeed = 3.0f;
    const unsigned int width = 1024;
    const unsigned int height = 262;

    // Set GLFW error callback
    glfwSetErrorCallback(handleGLFWError);

    // Initialize the library
    if(!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return EXIT_FAILURE;
    }

    // Prevent window being resized
    glfwWindowHint(GLFW_RESIZABLE, false);

    // Create a windowed mode window and its OpenGL context
    GLFWwindow *window = glfwCreateWindow(width, height, "Ant world", nullptr, nullptr);
    if(!window)
    {
        glfwTerminate();
        std::cerr << "Failed to create window" << std::endl;
        return EXIT_FAILURE;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    if(glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return EXIT_FAILURE;
    }

    // Enable VSync
    glfwSwapInterval(1);

    glDebugMessageCallback(handleGLError, nullptr);

    // Set clear colour to match matlab and enable depth test
    glClearColor(0.75f, 0.75f, 0.75f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glEnable(GL_TEXTURE_2D);

    // Create renderer - increasing cubemap size to improve quality in larger window
    // and pushing back clipping plane to reduce Z fighting
    AntWorld::Renderer renderer(512, 0.1);
    renderer.getWorld().load("../libantworld/world5000_gray.bin",
                             {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});

    // Load world, keeping texture sizes below 4096 and compressing textures on upload
    //renderer.getWorld().loadObj("object.obj",
    //                            0.1f, 4096, GL_COMPRESSED_RGB);


    // Create HID device for controlling movement
    HID::Joystick joystick(0.25f);

    // Get world bounds and initially centre agent in world
    const auto &worldMin = renderer.getWorld().getMinBound();
    const auto &worldMax = renderer.getWorld().getMaxBound();
    float x = worldMin[0] + ((worldMax[0] - worldMin[0]) * 0.5f);
    float y = worldMin[1] + ((worldMax[1] - worldMin[1]) * 0.5f);
    float z = worldMin[2] + ((worldMax[2] - worldMin[2]) * 0.5f);
    float yaw = 0.0f;
    float pitch = 0.0f;

    bool ant = true;
    double lastTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        // Poll joystick
        joystick.update();

        // Calculate time
        const double currentTime = glfwGetTime();
        const double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        char buffer[100];
        sprintf(buffer, "%d FPS", (int)std::round(1.0 / deltaTime));
        glfwSetWindowTitle(window, buffer);

        // Control yaw and pitch with left stick
        yaw += joystick.getState(HID::JAxis::LeftStickHorizontal) * deltaTime * turnSpeed;
        pitch += joystick.getState(HID::JAxis::LeftStickVertical) * deltaTime * turnSpeed;

        // Switch between human and ant mode using X
        if(joystick.isPressed(HID::JButton::X)) {
            ant = !ant;
        }


        // Use right trigger to control forward movement speed
        const float forwardMove = moveSpeed * deltaTime * joystick.getState(HID::JAxis::RightTrigger);

        // Calculate movement delta in 3D space
        x += forwardMove * sin(yaw * AntWorld::degreesToRadians) * cos(pitch * AntWorld::degreesToRadians);
        y += forwardMove * cos(yaw * AntWorld::degreesToRadians) * cos(pitch * AntWorld::degreesToRadians);
        z -= forwardMove * sin(pitch * AntWorld::degreesToRadians);

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render first person
        if(ant) {
            renderer.renderPanoramicView(x, y, z, yaw, pitch, 0.0f,
                                         0, 0, width, height);
        }
        else {
            renderer.renderFirstPersonView(x, y, z, yaw, pitch, 0.0f,
                                           0, 0, width, height);
        }

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }
    return EXIT_SUCCESS;
}