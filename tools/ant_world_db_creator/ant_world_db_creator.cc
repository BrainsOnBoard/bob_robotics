
// BoB robotics includes
#include "common/logging.h"
#include "common/pose.h"
#include "antworld/agent.h"
#include "antworld/common.h"
#include "antworld/renderer.h"
#include "antworld/route_continuous.h"
#include "navigation/image_database.h"
#include "video/opengl/opengl.h"

// Third-party includes
#include "third_party/path.h"

// OpenGL includes
#include <GL/glew.h>

// SFML includes
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <algorithm>
#include <string>
#include <fstream>
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

class AntWorldDatabaseCreator {
protected:
    ImageDatabase m_Database;
    sf::Window &m_Window;
    AntWorld::Renderer m_Renderer;
    AntWorld::AntAgent m_Agent;

    AntWorldDatabaseCreator(const std::string &databaseName,
                            sf::Window &window, const filesystem::path &executablePath)
      : m_Database(databaseName)
      , m_Window(window)
      , m_Renderer(256, 0.001, 1000.0, 360_deg)
      , m_Agent(window, m_Renderer, RenderSize)
    {
        BOB_ASSERT(m_Database.empty());

        // Create renderer
        m_Renderer.getWorld().load(executablePath / "../../resources/antworld/world5000_gray.bin",
                                   {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});
    }

    template<typename PoseVectorType, typename RecordOp>
    void run(const PoseVectorType &poses, RecordOp record)
    {
        // Host OpenCV array to hold pixels read from screen
        cv::Mat frame(RenderSize, CV_8UC3);

        for (auto it = poses.cbegin(); m_Window.isOpen() && it < poses.cend(); ++it) {
            // Update agent's position
            m_Agent.setPosition(it->x(), it->y(), AgentHeight);
            m_Agent.setAttitude(it->yaw(), 0_deg, 0_deg);

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
    GridDatabaseCreator(sf::Window &window, const filesystem::path &executablePath)
      : AntWorldDatabaseCreator("world5000_grid", window, executablePath)
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

        // Record image database
        run(gridRecorder.getPositions(), [&gridRecorder](const cv::Mat &image) { gridRecorder.record(image); });
    }
};

class RouteDatabaseCreator : AntWorldDatabaseCreator {
public:
    RouteDatabaseCreator(const std::string &databaseName,
                         sf::Window &window,
                         AntWorld::RouteContinuous &route,
                         const filesystem::path &executablePath)
      : AntWorldDatabaseCreator(databaseName, window, executablePath)
      , m_Route(route)
    {}

    void runForRoute()
    {
        const millimeter_t pathStep = 1_cm;

        // Make vector of agent's poses
        std::vector<Pose2<meter_t, degree_t>> poses;
        for (auto distance = 0_mm; distance < m_Route.getLength(); distance += pathStep) {
            poses.emplace_back(m_Route.getPose(distance));
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
    const auto executablePath = filesystem::path(argv[0]).parent_path();

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

        RouteDatabaseCreator creator(databaseName, *window, route, executablePath);
        creator.runForRoute();
    } else {
        GridDatabaseCreator creator(*window, executablePath);
        creator.runForGrid();
    }
}
