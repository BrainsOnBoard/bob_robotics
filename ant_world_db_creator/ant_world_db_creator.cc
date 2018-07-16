// Standard C++ includes
#include <string>
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
    const meter_t pathStep = 1_cm;
    const meter_t gridSpacing = 10_cm;
    const meter_t gridMax = 2.5_m; // gives a 5m^2 grid

    /*
     * I've set the width of the image to be the same as the (raw) unwrapped
     * images we get from the robot gantry, but the height is greater (cf. 58)
     * because I wanted to keep the aspect ratio as it was (200x40).
     *      -- AD
     */
    const unsigned int renderWidth = 720;
    const unsigned int renderHeight = 150;

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
    const bool followRoute = argc > 1;

    // Create route object and load route file specified by command line
    AntWorld::RouteContinuous route(0.2f, 800);

    // If we should be following a route
    std::ofstream csvStream;
    std::string routeTitle;
    filesystem::path savePath;
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

        savePath = routeTitle;
        if (!savePath.exists()) {
            filesystem::create_directory(savePath);
        }
        csvStream.open((savePath / (routeTitle + ".csv")).str());
    }
    else {
        savePath = "world5000_grid";
        if (!savePath.exists()) {
            filesystem::create_directory(savePath);
        }
        csvStream.open((savePath / "world5000_grid.csv").str());
    }

    // Write CSV header
    csvStream << "X [mm], Y [mm], Z [mm], Heading [degrees], Filename" << std::endl;

    // Create renderer
    AntWorld::Renderer renderer(256, 0.001, 1000.0, 360_deg);
    renderer.getWorld().load("../libantworld/world5000_gray.bin",
                             {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});

    // Get world bounds
    const auto &worldMinBound = renderer.getWorld().getMinBound();
    const auto &worldMaxBound = renderer.getWorld().getMaxBound();

    // Define the origin as the centre of the world, to nearest whole mm
    const millimeter_t originX = (worldMaxBound[0] - worldMinBound[0]) / 2.0;
    const millimeter_t originY = (worldMaxBound[1] - worldMinBound[1]) / 2.0;
    const Vector2m origin(round(originX), round(originY));

    // The extent of the grid is the origin +-gridMax
    const Vector2m worldMin(origin.X - gridMax, origin.Y - gridMax);
    const Vector2m worldMax(origin.X + gridMax, origin.Y + gridMax);

    // Create input to read snapshots from screen
    Video::OpenGL input(0, 0, renderWidth, renderHeight);

    // Host OpenCV array to hold pixels read from screen
    cv::Mat snapshot(renderHeight, renderWidth, CV_8UC3);

    size_t routePosition = 0;
    size_t currentGridX = 0;
    size_t currentGridY = 0;
    meter_t x, y;
    millimeter_t xMM, yMM;
    const millimeter_t zMM = 1_cm; // agent's height is fixed
    degree_t heading = 0_deg;

    // While the window isn't forcibly being closed
    while (!glfwWindowShouldClose(window)) {
        // If we should be following route, get position from route
        if(followRoute) {
            std::tie(x, y, heading) = route.getPosition(pathStep * routePosition);
        }
        else {
            x = worldMin.X + gridSpacing * currentGridX;
            y = worldMin.Y + gridSpacing * currentGridY;
        }

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render first person
        renderer.renderPanoramicView(x, y, zMM,
                                     heading, 0_deg, 0_deg,
                                     0, 0, renderWidth, renderHeight);

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Read snapshot
        input.readFrame(snapshot);

        // Convert to mm
        xMM = x;
        yMM = y;

        // Get image file name
        char filename[255];
        if(followRoute) {
            sprintf(filename, "%s_%04zu.png", routeTitle.c_str(), routePosition);
        }
        else {
            sprintf(filename, "world5000_grid_%05d_%05d_%05d.png",
                    (int) round(xMM), (int) round(y), (int) round(zMM));
        }

        // Write image file info to CSV file
        csvStream << xMM.value() << ", " << yMM.value() << ", " << zMM.value() << ", "
                  << heading.value() << ", " << filename << std::endl;

        // Write image file
        cv::flip(snapshot, snapshot, 0);
        cv::imwrite((savePath / filename).str(), snapshot);

        // Poll for and process events
        glfwPollEvents();

        // If we're following a route
        if(followRoute) {
            // Make next step
            routePosition++;

            // If we've gone over end of route, stop
            if(routePosition * pathStep > route.getLength()) {
                break;
            }
        }
        else {

            // Move to next X
            currentGridX++;

            // If we've reached the X edge of the world, move to start of next Y
            if(worldMin[0] + currentGridX * gridSpacing > worldMax[0]) {
                currentGridY++;
                currentGridX = 0;
            }

            // If we've reached the Y edge of the world, stop
            if(worldMin[1] + currentGridY * gridSpacing > worldMax[1]) {
                break;
            }
        }
    }

    csvStream.close(); // close CSV file handle
    return EXIT_SUCCESS;
}