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
    const float moveSpeed = 1.0f;
    const unsigned int width = 592;
    const unsigned int height = 152;

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
    GLFWwindow *window = glfwCreateWindow(width, 592 + height + 10, "Ant World", nullptr, nullptr);
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

    // Create renderer
    AntWorld::Renderer renderer(width, height);
    //renderer.loadWorld("../libantworld/world5000_gray.bin",
    //                   {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});

    // Load world, keeping texture sizes below 4096 and compressing textures on upload
    renderer.loadWorldObj("/home/j/jk/jk421/Documents/pier/pier_alex_smoothed_decimated_triangles.obj", 4096, GL_COMPRESSED_RGB);

    HID::Joystick joystick(0.25f);

    float antX = 5.0f;
    float antY = 5.0f;
    float antHeading = 270.0f;
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

        // Rotate ant using left stick's horizontal axis
        antHeading += joystick.getState(HID::JAxis::LeftStickHorizontal) * deltaTime * turnSpeed;

        // Move ant using left stick's vertical axis
        const float forwardMove = moveSpeed * deltaTime * joystick.getState(HID::JAxis::LeftStickVertical);
        antX -= forwardMove * sin(antHeading * AntWorld::degreesToRadians);
        antY -= forwardMove * cos(antHeading * AntWorld::degreesToRadians);

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render top down and ants eye view
        renderer.render(antX, antY, antHeading);

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }
    return EXIT_SUCCESS;
}