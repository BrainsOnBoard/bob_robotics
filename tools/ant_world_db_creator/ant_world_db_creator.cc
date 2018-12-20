
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

/*
* I've set the width of the image to be the same as the (raw) unwrapped
* images we get from the robot gantry, but the height is greater (cf. 58)
* because I wanted to keep the aspect ratio as it was (200x40).
*      -- AD
*/
const cv::Size RenderSize{ 720, 150 };
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
      , m_Agent(window, m_Renderer, RenderSize)
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
        cv::Mat frame(RenderSize, CV_8UC3);

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
        auto &metadata = recorder.getMetadataWriter();
        metadata << "camera" << m_Agent
                 << "needsUnwrapping" << false
                 << "isGreyscale" << false;
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
        Range xrange({worldMinBound[0], worldMaxBound[0]}, gridSpacing);
        Range yrange({worldMinBound[1], worldMaxBound[1]}, gridSpacing);
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
    auto window = AntWorld::AntAgent::initialiseWindow(RenderSize);

    if (argc > 1) {
        // Create route object and load route file specified by command line
        AntWorld::RouteContinuous route(0.2f, 800);
        route.load(argv[1]);

        // Get filename from route path
        std::string databaseName = filesystem::path(argv[1]).filename();

        // If it exists, remove extension
        const size_t pos = databaseName.find_last_of(".");
        if (pos != std::string::npos) {
            databaseName = databaseName.substr(0, pos);
        }

        RouteDatabaseCreator creator(databaseName, window.get(), route);
        creator.runForRoute();
    } else {
        GridDatabaseCreator creator(window.get());
        creator.runForGrid();
    }
}
