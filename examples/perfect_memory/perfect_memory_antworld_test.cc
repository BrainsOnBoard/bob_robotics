// Standard C++ includes
#include <iostream>

// BoB robotics includes
#include "navigation/antworld_rotater.h"
#include "navigation/perfect_memory.h"
#include "navigation/plot.h"

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;

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

template<typename T>
void
trainRoute(T &pm)
{
    // Load snapshots
    pm.trainRoute("../../tools/ant_world_db_creator/ant1_route1", true);
    std::cout << "Loaded " << pm.getNumSnapshots() << " snapshots" << std::endl;
}

int
main()
{
    /*
     * I've set the width of the image to be the same as the (raw) unwrapped
     * images we get from the robot gantry, but the height is greater (cf. 58)
     * because I wanted to keep the aspect ratio as it was (200x40).
     *      -- AD
     */
    const unsigned int renderWidth = 180;
    const unsigned int renderHeight = 50;

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
    GLFWwindow *window = glfwCreateWindow(renderWidth, renderHeight, "Ant world", nullptr, nullptr);
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
    renderer.getWorld().load("../../libantworld/world5000_gray.bin",
                             {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});

    // Create agent object
    AntWorld::AntAgent agent(window, renderer, renderWidth, renderHeight);
    agent.setPosition(5.5_m, 4_m, 10_mm);

    const cv::Size imSize(renderWidth, renderHeight);
    degree_t heading;

    {
        std::cout << "Using ant world rotater..." << std::endl;
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<>, BestMatchingSnapshot, AntWorldRotater> pm(imSize);
        trainRoute(pm);

        size_t snapshot;
        float difference;
        std::vector<std::vector<float>> allDifferences;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(agent, 2_deg);
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;

        // Plot RIDF
        plotRIDF(allDifferences[snapshot]);
        std::cout << std::endl;
    }

    {
        std::cout << "Using in silico rotater..." << std::endl;
        PerfectMemoryRotater<PerfectMemoryStore::RawImage<>, BestMatchingSnapshot, InSilicoRotater> pm(imSize);
        trainRoute(pm);

        agent.setAttitude(0_deg, 0_deg, 0_deg);
        cv::Mat fr;
        agent.readGreyscaleFrame(fr);

        size_t snapshot;
        float difference;
        std::vector<std::vector<float>> allDifferences;
        std::tie(heading, snapshot, difference, allDifferences) = pm.getHeading(fr);
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "Best-matching snapshot: #" << snapshot << std::endl;
        std::cout << "Difference score: " << difference << std::endl;

        // Plot RIDF
        plotRIDF(allDifferences[snapshot]);
        std::cout << std::endl;
    }
}