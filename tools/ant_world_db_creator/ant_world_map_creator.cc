// BoB robotics includes
#include "common/logging.h"
#include "antworld/common.h"
#include "antworld/renderer.h"
#include "antworld/route_continuous.h"
#include "video/opengl.h"

// Third-party includes
#include "third_party/path.h"

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// Standard C++ includes
#include <iostream>

using namespace BoBRobotics;

// Anonymous namespace
namespace
{
void handleGLFWError(int errorNumber, const char *message)
{
    std::cerr << "GLFW error number:" << errorNumber << ", message:" << message << std::endl;
}

void handleGLError(GLenum, GLenum, GLuint, GLenum, GLsizei, const GLchar *message, const void *)
{
    throw std::runtime_error(message);
}

}

int main()
{
    const unsigned int renderWidth = 1050;
    const unsigned int renderHeight = 1050;

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
    GLFWwindow *window = glfwCreateWindow(renderWidth, renderHeight, "Ant world", nullptr, nullptr);
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
    glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glEnable(GL_TEXTURE_2D);

    // **NOTE** because 1050 isn't a multiple of 4 - need to tweak pack alignement
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // Create renderer
    AntWorld::Renderer renderer;
    renderer.getWorld().load("../../include/antworld/world5000_gray.bin",
                             {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});

    // Create input to read snapshots from screen
    Video::OpenGL input({ renderWidth, renderHeight });

    // Host OpenCV array to hold pixels read from screen
    cv::Mat map(renderHeight, renderWidth, CV_8UC3);


    // Clear colour and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render first person
    renderer.renderTopDownView(0, 0, renderWidth, renderHeight);

    // Swap front and back buffers
    glfwSwapBuffers(window);

    // Read snapshot
    input.readFrame(map);

    cv::imwrite("world5000.png", map);

    return EXIT_SUCCESS;
}
