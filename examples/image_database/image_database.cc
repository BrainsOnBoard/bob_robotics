// BoB robotics includes
#include "navigation/image_database.h"
#include "video/randominput.h"

// Third-party includes
#include "third_party/units.h"
#include "third_party/path.h"

// Standard C++ includes
#include <iostream>

using namespace BoBRobotics;
using namespace units::literals;

auto getNewDatabaseName()
{
    filesystem::path databaseName;
    int databaseCount = 0;
    do {
        std::stringstream ss;
        ss << "database" << ++databaseCount;
        databaseName = ss.str();
    } while (databaseName.exists());
    return databaseName;
}

struct Agent
{
    template<class PositionType, class Func>
    bool moveToSync(const PositionType &position, Func)
    {
        std::cout << "Position: " << position << std::endl;
        return true;
    }
};

int main()
{
    /*
     * These parameters give the option of alternating the x and y directions.
     * They can be ommitted.
     */
    constexpr bool alternateX = true, alternateY = false;

    constexpr Navigation::Range xrange({ 0_mm, 50_mm }, 10_mm);
    constexpr Navigation::Range yrange({ 0_mm, 60_mm }, 10_mm);
    constexpr Navigation::Range zrange({ 10_mm, 30_mm }, 10_mm);

    // Fake camera device
    Video::RandomInput<> video({ 200, 100 });

    Navigation::ImageDatabase database(getNewDatabaseName());
    auto recorder = database.getGridRecorder<alternateX, alternateY>(xrange, yrange, zrange);
    Agent agent;
    recorder.run(agent, video);
}
