
// BoB robotics includes
#include "libantworld/agent.h"
#include "libantworld/common.h"
#include "libantworld/renderer.h"
#include "libantworld/route_continuous.h"
#include "navigation/image_database.h"
#include "video/opengl.h"

// Third-party includes
#include "third_party/path.h"

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// Standard C++ includes
#include <algorithm>
#include <string>
#include <fstream>
#include <iostream>
#include <tuple>
#include <vector>

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::math;

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

/*
* I've set the width of the image to be the same as the (raw) unwrapped
* images we get from the robot gantry, but the height is greater (cf. 58)
* because I wanted to keep the aspect ratio as it was (200x40).
*      -- AD
*/
const unsigned RenderWidth = 720;
const unsigned RenderHeight = 150;

using Pose = std::tuple<meter_t, meter_t, degree_t>;

class AntWorldDatabaseCreator {
protected:
    ImageDatabase m_Database;
    GLFWwindow *m_Window;
    AntWorld::Renderer m_Renderer;
    AntWorld::AntAgent m_Agent;

    AntWorldDatabaseCreator(const std::string &databaseName, GLFWwindow *window)
      : m_Database(databaseName)
      , m_Window(window)
      , m_Renderer(256, 0.001, 1000.0, 360_deg)
      , m_Agent(window, m_Renderer, RenderWidth, RenderHeight)
    {
        BOB_ASSERT(m_Database.empty());

        // Create renderer
        m_Renderer.getWorld().load("../../libantworld/world5000_gray.bin",
                                   {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});
    }

    template<typename RecordOp>
    void run(const std::vector<Pose> &poses, RecordOp record)
    {
        // Host OpenCV array to hold pixels read from screen
        cv::Mat frame(RenderHeight, RenderWidth, CV_8UC3);

        for (auto it = poses.cbegin(); !glfwWindowShouldClose(m_Window) && it < poses.cend(); ++it) {
            meter_t x, y;
            degree_t heading;
            std::tie(x, y, heading) = *it;

            // Update agent's position
            m_Agent.setPosition(x, y, AgentHeight);
            m_Agent.setAttitude(heading, 0_deg, 0_deg);

            // Get current view
            m_Agent.readFrameSync(frame);

            // Write to image database
            record(frame);
        }
    }

    void addMetadata(ImageDatabase::Recorder &recorder)
    {
        // Record "camera" info
        recorder.getMetadataWriter() << "camera" << m_Agent << "needsUnwrapping" << false;
    }

    const millimeter_t AgentHeight = 1_cm;
};

class GridDatabaseCreator : AntWorldDatabaseCreator {
public:
    GridDatabaseCreator(GLFWwindow *window)
      : AntWorldDatabaseCreator("world5000_grid", window)
    {}

    void runForGrid()
    {
        const millimeter_t gridSpacing = 10_cm;

        // Get world bounds
        const auto &worldMinBound = m_Renderer.getWorld().getMinBound();
        const auto &worldMaxBound = m_Renderer.getWorld().getMaxBound();

        // Make GridRecorder
        Range xrange;
        xrange.begin = worldMinBound[0];
        xrange.end = worldMaxBound[0];
        xrange.separation = gridSpacing;
        Range yrange;
        yrange.begin = worldMinBound[1];
        yrange.end = worldMaxBound[1];
        yrange.separation = gridSpacing;
        auto gridRecorder = m_Database.getGridRecorder(xrange, yrange, AgentHeight);
        addMetadata(gridRecorder);

        // Convert positions to poses
        const auto positions = gridRecorder.getPositions();
        std::vector<Pose> poses;
        poses.reserve(positions.size());
        const auto pos2pose = [](const auto &pos) {
            return std::make_tuple<meter_t, meter_t, degree_t>(pos[0], pos[1], 0_deg);
        };
        std::transform(positions.cbegin(), positions.cend(), std::back_inserter(poses), pos2pose);

        // Record image database
        run(poses, [&gridRecorder](const cv::Mat &image) { gridRecorder.record(image); });
    }
};

class RouteDatabaseCreator : AntWorldDatabaseCreator {
public:
    RouteDatabaseCreator(const std::string &databaseName, GLFWwindow *window,
                         AntWorld::RouteContinuous &route)
      : AntWorldDatabaseCreator(databaseName, window)
      , m_Route(route)
    {}

    void runForRoute()
    {
        const millimeter_t pathStep = 1_cm;

        // Make vector of agent's poses
        std::vector<Pose> poses;
        for (auto distance = 0_mm; distance < m_Route.getLength(); distance += pathStep) {
            poses.emplace_back(m_Route.getPosition(distance));
        }

        // Record image database
        auto routeRecorder = m_Database.getRouteRecorder();
        addMetadata(routeRecorder);

        run(poses, [&routeRecorder, this](const cv::Mat &image) {
            const auto pos = m_Agent.getPosition();
            routeRecorder.record({pos[0], pos[1], pos[2]}, m_Agent.getAttitude()[0], image);
        });
    }

private:
    AntWorld::RouteContinuous &m_Route;
};

int
main(int argc, char **argv)
{
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
    GLFWwindow *window = glfwCreateWindow(RenderWidth, RenderHeight, "Ant world", nullptr, nullptr);
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

    if (argc > 1) {
        // Create route object and load route file specified by command line
        AntWorld::RouteContinuous route(0.2f, 800);
        if (!route.load(argv[1])) {
            return 1;
        }

        // Get filename from route path
        std::string databaseName = filesystem::path(argv[1]).filename();

        // If it exists, remove extension
        const size_t pos = databaseName.find_last_of(".");
        if (pos != std::string::npos) {
            databaseName = databaseName.substr(0, pos);
        }

        RouteDatabaseCreator creator(databaseName, window, route);
        creator.runForRoute();
    } else {
        GridDatabaseCreator creator(window);
        creator.runForGrid();
    }
}