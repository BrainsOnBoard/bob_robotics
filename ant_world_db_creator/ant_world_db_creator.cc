// Standard C++ includes
#include <fstream>
#include <iostream>

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// GeNN robotics includes
#include "../third_party/path.h"
#include "../video/opengl.h"

// Libantworld includes
#include "../libantworld/common.h"
#include "../libantworld/renderer.h"
#include "../libantworld/route_continuous.h"

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

int main(int argc, char *argv[])
{
    const float pathStepM = 1.0f / 100.0f;
    const float gridSizeM = 100.0f / 100.0f;

    const unsigned int renderWidth = 180;
    const unsigned int renderHeight = 50;

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

    // Should we follow route or grid
    const bool followRoute = (argc > 1);

    // Create route object and load route file specified by command line
    AntWorld::RouteContinuous route(0.2f, 800);

    // If we should be following a route
    std::ofstream csvStream;
    std::string routeTitle;
    if(followRoute) {
        // Load route
        route.load(argv[1]);

        // Get filename from route path
        routeTitle = filesystem::path(argv[1]).filename();

        // If it exists, remove extension
        const size_t pos = routeTitle.find_last_of(".");
        if (pos != std::string::npos) {
             routeTitle = routeTitle.substr(0, pos);
        }

        csvStream.open(routeTitle + ".csv");
        csvStream << "X [mm], Y [mm], Heading [degrees], Filename" << std::endl;
    }
    else {
        csvStream.open("grid.csv");
        csvStream << "X [mm], Y [mm], Filename" << std::endl;
    }

    // Create renderer
    AntWorld::Renderer renderer;
    renderer.getWorld().load("../libantworld/world5000_gray.bin",
                             {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});

    // Get world bounds
    const auto &worldMin = renderer.getWorld().getMinBound();
    const auto &worldMax = renderer.getWorld().getMaxBound();

    // Create input to read snapshots from screen
    Video::OpenGL input(0, 0, renderWidth, renderHeight);


    // Host OpenCV array to hold pixels read from screen
    cv::Mat snapshot(renderHeight, renderWidth, CV_8UC3);


    // While the window isn't forcibly being closed
    size_t routePosition = 0;
    size_t currentGridX = 0;
    size_t currentGridY = 0;
    while (!glfwWindowShouldClose(window)) {
        // If we should be following route, get position from route
        float x = 0.0f;
        float y = 0.0f;
        float heading = 0.0f;
        if(followRoute) {
            std::tie(x, y, heading) = route.getPosition((float)routePosition * pathStepM);
        }
        else {
            x = worldMin[0] + ((float)currentGridX * gridSizeM);
            y = worldMin[1] + ((float)currentGridY * gridSizeM);
        }

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render first person
        renderer.renderPanoramicView(x, y, 0.01f,
                                     heading, 0.0f, 0.0f,
                                     0, 0, renderWidth, renderHeight);

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Read snapshot
        input.readFrame(snapshot);

        char filename[255];
        if(followRoute) {
            sprintf(filename, "%s_%zu.png", routeTitle.c_str(), routePosition);

            csvStream << x << ", " << y << ", " << heading << ", " << filename << std::endl;
        }
        else {
            sprintf(filename, "grid_%zu_%zu.png", currentGridX, currentGridY);

            csvStream << x << ", " << y << ", " << filename << std::endl;
        }

        cv::flip(snapshot, snapshot, 0);
        cv::imwrite(filename, snapshot);

        // Poll for and process events
        glfwPollEvents();

        // If we're following a route
        if(followRoute) {
            // Make next step
            routePosition++;

            // If we've gone over end of route, stop
            if(((float)routePosition * pathStepM) > route.getLength()) {
                break;
            }
        }
        else {

            // Move to next X
            currentGridX++;

            // If we've reached the X edge of the world, move to start of next Y
            if((worldMin[0] + ((float)currentGridX * gridSizeM)) > worldMax[0]) {
                currentGridY++;
                currentGridX = 0;
            }

            // If we've reached the Y edge of the world, stop
            if((worldMin[1] + ((float)currentGridY * gridSizeM)) > worldMax[1]) {
                break;
            }
        }
    }
    return EXIT_SUCCESS;
}