
// BoB robotics includes
#include "plog/Log.h"
#include "common/path.h"
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

// Standard C includes
#include <cstring>

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
    const meter_t m_AgentHeight;
    const bool m_OldAntWorld;

    AntWorldDatabaseCreator(const filesystem::path &databaseName,
                            bool oldAntWorld,
                            meter_t agentHeight,
                            sf::Window &window)
      : m_Database(databaseName)
      , m_Window(window)
      , m_Renderer(512, 0.1)
      , m_Agent(window, m_Renderer, RenderSize)
      , m_AgentHeight(agentHeight)
      , m_OldAntWorld(oldAntWorld)
    {
        BOB_ASSERT(m_Database.empty());

        // Create renderer
        const auto antWorldPath = Path::getResourcesPath() / "antworld";
        if (oldAntWorld) {
            m_Renderer.getWorld().load(antWorldPath / "world5000_gray.bin",
                                       { 0.0f, 1.0f, 0.0f },
                                       { 0.898f, 0.718f, 0.353f });
        } else {
            m_Renderer.getWorld().loadObj(antWorldPath / "seville_vegetation_downsampled.obj");
        }
    }

    template<typename PoseVectorType, typename RecordOp>
    void run(const PoseVectorType &poses, RecordOp record)
    {
        // Host OpenCV array to hold pixels read from screen
        cv::Mat frame(RenderSize, CV_8UC3);

        for (auto it = poses.cbegin(); m_Window.isOpen() && it < poses.cend(); ++it) {
            // Process window events
            sf::Event event;
            while (m_Window.pollEvent(event)) {
                // Close window: exit
                if (event.type == sf::Event::Closed) {
                    m_Window.close();
                }
            }
            
            // Update agent's position
            m_Agent.setPosition(it->x(), it->y(), m_AgentHeight);
            m_Agent.setAttitude(it->yaw(), 0_deg, 0_deg);
            LOGD << "Current pose: " << m_Agent.getPose();

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
};

class GridDatabaseCreator : AntWorldDatabaseCreator {
public:
    GridDatabaseCreator(const filesystem::path &databaseName, bool oldAntWorld,
                        sf::Window &window)
      : AntWorldDatabaseCreator(databaseName, oldAntWorld,
                                oldAntWorld ? 0.01_m : 1.5_m, window)
    {}

    void runForGrid()
    {
        constexpr auto gridSpacing = 10_cm;

        Range xrange, yrange;
        if (m_OldAntWorld) {
            // Get world bounds
            const auto &worldMinBound = m_Renderer.getWorld().getMinBound();
            xrange.begin = worldMinBound[0];
            yrange.begin = worldMinBound[1];
            const auto &worldMaxBound = m_Renderer.getWorld().getMaxBound();
            xrange.end = worldMaxBound[0];
            yrange.end = worldMaxBound[1];
        } else {
            xrange.begin = yrange.begin = -2.5_m;
            xrange.end = yrange.end = 2.5_m;
        }
        xrange.separation = yrange.separation = gridSpacing;

        // Make GridRecorder
        auto gridRecorder = m_Database.getGridRecorder(xrange, yrange, {m_AgentHeight});
        addMetadata(gridRecorder);

        // Record image database
        run(gridRecorder.getPositions(), [&gridRecorder](const cv::Mat &image) { gridRecorder.record(image); });
    }
};

class RouteDatabaseCreator : AntWorldDatabaseCreator {
public:
    RouteDatabaseCreator(const filesystem::path &databaseName,
                         bool oldAntWorld,
                         sf::Window &window,
                         AntWorld::RouteContinuous &route)
      : AntWorldDatabaseCreator(databaseName, oldAntWorld,
                                oldAntWorld ? 0.01_m : 1.8_m, window)
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
            const auto pos = m_Agent.getPose().position();
            routeRecorder.record({pos[0], pos[1], pos[2]}, m_Agent.getPose().attitude()[0], image);
        });
    }

private:
    AntWorld::RouteContinuous &m_Route;
};

int bobMain(int argc, char **argv)
{
    const auto currentDir = filesystem::path{ argv[0] }.parent_path();
    auto window = AntWorld::AntAgent::initialiseWindow(RenderSize);

    // Allow for using the old, lower-res ant world
    bool oldAntWorld = false;
    if (argc > 1 && strcmp(argv[1], "--old-ant-world") == 0) {
        oldAntWorld = true;
        argc--;
        argv++;
    }

    if (argc > 1) {
        // Remaining arguments are paths to route files
        do {
            // Create route object and load route file specified by command line
            AntWorld::RouteContinuous route(0.2f, 800);
            route.load(argv[1]);

            // Get filename from route path
            std::string databaseName = filesystem::path(argv[1]).filename();

            // Remove extension
            const size_t pos = databaseName.find_last_of(".");
            if (pos != std::string::npos) {
                databaseName = databaseName.substr(0, pos);
            }

            RouteDatabaseCreator creator(currentDir / databaseName, oldAntWorld, *window, route);
            creator.runForRoute();

            argv++;
        } while (--argc > 1);
    } else {
        GridDatabaseCreator creator(currentDir / "world5000_grid", oldAntWorld, *window);
        creator.runForGrid();
    }

    return EXIT_SUCCESS;
}
